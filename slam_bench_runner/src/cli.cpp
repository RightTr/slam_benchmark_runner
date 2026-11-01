#include "cli.hpp"

#include <cerrno>
#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <ostream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <unistd.h>
#include <termios.h>
#include <vector>

namespace slam_bench::cli {
namespace {
volatile std::sig_atomic_t interrupt_requested = 0;

void handleInterrupt(int) { interrupt_requested = 1; }

bool colorEnabled() {
  const char* term = ::getenv("TERM");
  return ::isatty(STDOUT_FILENO) && ::getenv("NO_COLOR") == nullptr &&
    term != nullptr && std::string_view(term) != "dumb";
}

enum class InputResult { Line, Interrupt, EndOfInput, RunError, RunFinished };
void prompt(const char* label = "bench> ");
InputResult readLine(std::string& line, const char* prompt_label = "bench> ", bool use_history = true,
                     const std::function<bool()>& on_idle = {});

class RawTerminal {
 public:
  RawTerminal() {
    if (!::isatty(STDIN_FILENO) || ::tcgetattr(STDIN_FILENO, &saved_) != 0) return;
    termios raw = saved_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    active_ = ::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
  }
  ~RawTerminal() { restore(); }
  bool active() const { return active_; }
  void restore() {
    if (active_) {
      ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
      active_ = false;
    }
  }
 private:
  termios saved_{};
  bool active_{};
};

enum class Key { Up, Down, Enter, Back, Interrupt, None, Other };

Key readKey() {
  if (interrupt_requested) {
    interrupt_requested = 0;
    return Key::Interrupt;
  }
  pollfd input{STDIN_FILENO, POLLIN, 0};
  int ready;
  do { ready = ::poll(&input, 1, 100); } while (ready < 0 && errno == EINTR && !interrupt_requested);
  if (interrupt_requested) {
    interrupt_requested = 0;
    return Key::Interrupt;
  }
  if (ready == 0) return Key::None;
  if (ready < 0 || !(input.revents & POLLIN)) return Key::Other;
  char c{};
  if (::read(STDIN_FILENO, &c, 1) != 1) return Key::Back;
  if (c == '\n' || c == '\r') return Key::Enter;
  if (c == 27) {
    pollfd escape{STDIN_FILENO, POLLIN, 0};
    if (::poll(&escape, 1, 60) <= 0) return Key::Back;
    char bracket{};
    if (::read(STDIN_FILENO, &bracket, 1) != 1 || bracket != '[') return Key::Back;
    char direction{};
    if (::read(STDIN_FILENO, &direction, 1) != 1) return Key::Back;
    if (direction == 'A') return Key::Up;
    if (direction == 'B') return Key::Down;
    return Key::Other;
  }
  if (c == 'q' || c == 'Q') return Key::Back;
  return Key::Other;
}

InputResult readLine(std::string& line, const char* prompt_label, bool use_history,
                     const std::function<bool()>& on_idle) {
  static std::vector<std::string> history;
  RawTerminal raw;
  line.clear();
  if (!raw.active()) {
    while (true) {
      if (interrupt_requested) {
        interrupt_requested = 0;
        return InputResult::Interrupt;
      }
      if (on_idle && on_idle()) { line.clear(); return InputResult::RunError; }
      pollfd input{STDIN_FILENO, POLLIN, 0};
      const int ready = ::poll(&input, 1, 100);
      if (ready < 0) {
        if (errno == EINTR) continue;
        return InputResult::EndOfInput;
      }
      if (ready == 0) continue;
      if (input.revents & (POLLERR | POLLNVAL)) return InputResult::EndOfInput;
      if (input.revents & (POLLIN | POLLHUP)) {
        char ch{};
        if (::read(STDIN_FILENO, &ch, 1) != 1) return InputResult::EndOfInput;
        if (ch == '\n' || ch == '\r') {
          if (use_history && !line.empty() && (history.empty() || history.back() != line)) {
            history.push_back(line);
            if (history.size() > 100) history.erase(history.begin());
          }
          return InputResult::Line;
        }
        line.push_back(ch);
      }
    }
  }

  size_t history_index = history.size();
  std::string draft;
  auto redraw = [&] {
    std::cout << "\r\033[2K";
    prompt(prompt_label);
    std::cout << line << std::flush;
  };
  while (true) {
    if (interrupt_requested) {
      interrupt_requested = 0;
      line.clear();
      return InputResult::Interrupt;
    }
    if (on_idle && on_idle()) { line.clear(); return InputResult::RunError; }
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const int ready = ::poll(&input, 1, 100);
    if (ready < 0) {
      if (errno == EINTR) continue;
      return InputResult::EndOfInput;
    }
    if (ready == 0) continue;
    if (input.revents & (POLLERR | POLLNVAL)) return InputResult::EndOfInput;
    if (!(input.revents & POLLIN)) continue;
    char ch{};
    if (::read(STDIN_FILENO, &ch, 1) != 1) return InputResult::EndOfInput;
    if (ch == '\r' || ch == '\n') {
      std::cout << "\r\n" << std::flush;
      if (use_history && !line.empty() && (history.empty() || history.back() != line)) {
        history.push_back(line);
        if (history.size() > 100) history.erase(history.begin());
      }
      return InputResult::Line;
    }
    if (ch == 3) {
      line.clear();
      return InputResult::Interrupt;
    }
    if (ch == 4 && line.empty()) return InputResult::EndOfInput;
    if (ch == 127 || ch == 8) {
      if (!line.empty()) { line.pop_back(); std::cout << "\b \b" << std::flush; }
      continue;
    }
    if (ch == 27) {
      pollfd escape{STDIN_FILENO, POLLIN, 0};
      if (::poll(&escape, 1, 60) <= 0) continue;
      char bracket{}, direction{};
      if (::read(STDIN_FILENO, &bracket, 1) != 1 || bracket != '[' ||
          ::read(STDIN_FILENO, &direction, 1) != 1) continue;
      if (use_history && direction == 'A' && !history.empty()) {
        if (history_index == history.size()) draft = line;
        if (history_index > 0) --history_index;
        line = history[history_index];
        redraw();
      } else if (use_history && direction == 'B' && history_index < history.size()) {
        ++history_index;
        line = history_index == history.size() ? draft : history[history_index];
        redraw();
      }
      continue;
    }
    if (ch >= 32) { line.push_back(ch); std::cout << ch << std::flush; }
  }
}

enum class MenuResult { Confirmed, Cancelled, Interrupt };
void banner();

void redrawTestPicker(int selected) {
  if (::isatty(STDOUT_FILENO)) std::cout << "\033[2J\033[H";
  banner();
  std::cout << paint("  ╭─ Select a test sequence ────────────────────────────╮", "38;5;45") << '\n';
  for (int i = 1; i <= 6; ++i) {
    const bool active = i == selected;
    const std::string marker = active ? "  › Sequence " : "    Sequence ";
    const std::string content = marker + (i < 10 ? "0" : "") + std::to_string(i);
    std::cout << "  │" << (active ? paint(content, "1;38;5;45") : content)
              << std::string(54 - content.size(), ' ') << "│\n";
  }
  std::cout << paint("  ╰──────────────────────────────────────────────────────╯", "38;5;45") << '\n'
            << "  Use " << paint("↑ / ↓", "1;38;5;45") << " to move, " << paint("Enter", "1;38;5;82")
            << " to start, " << paint("Esc", "1;38;5;214") << " to return.\n" << std::flush;
}

MenuResult chooseTestSequence(int& selected) {
  RawTerminal raw;
  if (!raw.active()) {
    std::cout << "\n  Select test sequence:\n";
    for (int i = 1; i <= 6; ++i) std::cout << "    [" << i << "] Sequence 0" << i << '\n';
    prompt("sequence (1-6, blank cancels) › ");
    std::string answer;
    const auto result = readLine(answer,"sequence (1-6, blank cancels) › ",false);
    if (result == InputResult::Interrupt) return MenuResult::Interrupt;
    if (result == InputResult::EndOfInput || answer.empty()) return MenuResult::Cancelled;
    try { selected = std::stoi(answer); } catch (...) { return MenuResult::Cancelled; }
    return selected >= 1 && selected <= 6 ? MenuResult::Confirmed : MenuResult::Cancelled;
  }

  int choice = 1;
  bool redraw = true;
  while (true) {
    if (redraw) {
      redrawTestPicker(choice);
      redraw = false;
    }
    const Key key = readKey();
    if (key == Key::Interrupt) {
      raw.restore();
      return MenuResult::Interrupt;
    }
    if (key == Key::Back) {
      raw.restore();
      std::cout << "\033[?25h\n  Test selection cancelled.\n\n";
      return MenuResult::Cancelled;
    }
    if (key == Key::Up) { choice = choice == 1 ? 6 : choice - 1; redraw = true; }
    if (key == Key::Down) { choice = choice == 6 ? 1 : choice + 1; redraw = true; }
    if (key == Key::Enter) {
      selected = choice;
      raw.restore();
      std::cout << "\033[?25h" << paint(std::string("  Selected Sequence ") + (choice < 10 ? "0" : "") + std::to_string(choice), "1;38;5;82") << "\n\n";
      return MenuResult::Confirmed;
    }
  }
}

MenuResult confirmEvaluationStart() {
  std::cout << '\n' << paint("  ╭─ Evaluation run ────────────────────────────────────╮", "38;5;45") << '\n'
            << "  │ Six sequences will run in a randomized order.        │\n"
            << paint("  ╰──────────────────────────────────────────────────────╯", "38;5;45") << '\n';
  RawTerminal raw;
  if (!raw.active()) {
    prompt("press Enter to start · blank line › ");
    std::string answer;
    const auto result = readLine(answer,"press Enter to start · blank line › ",false);
    if (result == InputResult::Interrupt) return MenuResult::Interrupt;
    return result == InputResult::Line && answer.empty() ? MenuResult::Confirmed : MenuResult::Cancelled;
  }
  std::cout << "  Press " << paint("Enter", "1;38;5;82") << " to start or "
            << paint("Esc", "1;38;5;214") << " to cancel.\n";
  while (true) {
    const Key key = readKey();
    if (key == Key::Interrupt) { raw.restore(); return MenuResult::Interrupt; }
    if (key == Key::Enter) { raw.restore(); return MenuResult::Confirmed; }
    if (key == Key::Back) { raw.restore(); std::cout << "  Evaluation cancelled.\n\n"; return MenuResult::Cancelled; }
  }
}

MenuResult promptStudentId(std::string& student_id) {
  while (true) {
    prompt("Student ID (13 digits) › ");
    const auto result=readLine(student_id,"Student ID (13 digits) › ",false);
    if (result==InputResult::Interrupt) return MenuResult::Interrupt;
    if (result==InputResult::EndOfInput) return MenuResult::Cancelled;
    const bool valid=student_id.size()==13 &&
      std::all_of(student_id.begin(),student_id.end(),[](unsigned char c){return std::isdigit(c);});
    if (valid) return MenuResult::Confirmed;
    std::cout << paint("  Enter exactly 13 digits.\n", "38;5;203");
  }
}

void prompt(const char* label) {
  std::cout << paint(std::string("  ") + label, "1;38;5;45") << std::flush;
}

bool confirmExit(bool active_run) {
  const std::string title = active_run ? "Stop the active run and exit?" : "Exit SLAM-Bench?";
  std::cout << '\n' << paint("  ╭─ Confirm exit ──────────────────────────────────────╮", "38;5;214") << '\n'
            << "  │ " << title << std::string(53 - title.size(), ' ') << "│\n"
            << paint("  ╰──────────────────────────────────────────────────────╯", "38;5;214") << '\n'
            << "  Choose " << paint("y", "1;38;5;82") << " to confirm, " << paint("n", "1;38;5;203")
            << " to return. Default is no.\n";
  prompt("confirm [y/N] › ");
  std::string answer;
  const auto result = readLine(answer,"confirm [y/N] › ",false);
  if (result == InputResult::Interrupt) {
    std::cout << "\n  Exit cancelled.\n";
    return false;
  }
  if (result == InputResult::EndOfInput) return true;
  for (auto& c : answer) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (answer == "y" || answer == "yes") return true;
  std::cout << "  Continuing session.\n\n";
  return false;
}

void banner() {
  constexpr int width = 116;
  constexpr int height = 84;
  std::array<std::array<std::uint8_t, width>, height> canvas{};
  struct Point { double x, y; };

  auto disc = [&](double cx, double cy, double radius, std::uint8_t ink) {
    const int x0 = std::max(0, static_cast<int>(std::floor(cx - radius)));
    const int x1 = std::min(width - 1, static_cast<int>(std::ceil(cx + radius)));
    const int y0 = std::max(0, static_cast<int>(std::floor(cy - radius)));
    const int y1 = std::min(height - 1, static_cast<int>(std::ceil(cy + radius)));
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x)
        if ((x - cx)*(x - cx) + (y - cy)*(y - cy) <= radius*radius)
          canvas[y][x] = ink;
  };
  auto segment = [&](Point a, Point b, double line_width, std::uint8_t ink) {
    const int steps = std::max(1, static_cast<int>(std::hypot(a.x-b.x, a.y-b.y) * 2));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      disc(a.x + t*(b.x-a.x), a.y + t*(b.y-a.y), line_width/2, ink);
    }
  };
  auto bezier = [&](Point a, Point b, Point c, Point d) {
    for (int i = 0; i <= 160; ++i) {
      const double t = i/160.0, u = 1.0-t;
      const double x = u*u*u*a.x + 3*u*u*t*b.x + 3*u*t*t*c.x + t*t*t*d.x;
      const double y = u*u*u*a.y + 3*u*u*t*b.y + 3*u*t*t*c.y + t*t*t*d.y;
      disc(x, y, 4.1, 2);
    }
  };

  segment({11,25},{11,8},6,1);
  segment({11,8},{30,8},6,1);
  segment({105,57},{105,74},6,1);
  segment({105,74},{86,74},6,1);
  bezier({20,66},{39,66},{63,67},{72,62});
  bezier({72,62},{88,52},{72,45},{57,43});
  bezier({57,43},{32,39},{35,28},{48,28});
  bezier({48,28},{61,28},{70,29},{77,29});
  disc(20,66,8,2);
  disc(20,66,3.4,0);
  disc(88,19,12,3);
  for (int y = 19; y <= 43; ++y) {
    const double half_width = 12.0*(43.0-y)/(43.0-19.0);
    for (int x = std::max(0,int(std::floor(88-half_width)));
         x <= std::min(width-1,int(std::ceil(88+half_width))); ++x)
      if (std::abs(x-88.0) <= half_width) canvas[y][x] = 3;
  }
  // Hollow center of the destination pin; oversized for clear Braille rendering.
  disc(88,19,7.0,0);

  constexpr std::uint8_t dots[4][2] = {{0x01,0x08},{0x02,0x10},{0x04,0x20},{0x40,0x80}};
  const bool color = colorEnabled();
  auto paintInk = [](std::uint8_t ink) {
    switch (ink) {
      case 1: std::cout << "\033[38;2;92;81;74m"; break;
      case 2: std::cout << "\033[38;2;214;111;72m"; break;
      case 3: std::cout << "\033[38;2;239;68;68m"; break;
      default: std::cout << "\033[0m"; break;
    }
  };

  std::cout << '\n';
  for (int y = 0; y < height; y += 4) {
    std::cout << "  ";
    std::uint8_t last_ink = 0;
    for (int x = 0; x < width; x += 2) {
      std::uint8_t mask = 0;
      int counts[4]{};
      for (int dy = 0; dy < 4; ++dy)
        for (int dx = 0; dx < 2; ++dx) {
          const auto ink = canvas[y+dy][x+dx];
          if (ink) { mask |= dots[dy][dx]; ++counts[ink]; }
        }
      std::uint8_t dominant = 0;
      for (std::uint8_t ink = 1; ink <= 3; ++ink)
        if (counts[ink] > counts[dominant]) dominant = ink;
      if (color && dominant != last_ink) paintInk(dominant);
      last_ink = dominant;
      if (!mask) { std::cout << ' '; continue; }
      const std::uint32_t cp = 0x2800u + mask;
      std::cout << static_cast<char>(0xE0u | (cp >> 12))
                << static_cast<char>(0x80u | ((cp >> 6) & 0x3Fu))
                << static_cast<char>(0x80u | (cp & 0x3Fu));
      }
    if (color) std::cout << "\033[0m";
    std::cout << '\n';
  }
  std::cout << '\n';
  if (color) std::cout << "\033[1;38;2;214;111;72m";
  std::cout << "    SLAM BENCH";
  if (color) std::cout << "\033[0m";
  std::cout << "  /  RUNNER  ·  v1.0\n";
  std::cout << "    RGB-D SLAM evaluation  ·  Test / Evaluation\n\n";
}

void help() {
  std::cout << paint("  MODES", "1;38;5;45") << "  "
            << paint("test", "1;38;5;82") << " (pick with ↑/↓)  "
            << paint("eval", "1;38;5;214") << " (Enter starts random sequence)\n"
            << paint("  RUN", "1;38;5;45") << "    "
            << paint("pause", "1;38;5;214") << "  "
            << paint("resume", "1;38;5;82") << "  "
            << paint("cancel", "1;38;5;203") << "  "
            << paint("status", "1;38;5;45") << '\n'
            << paint("  EXIT", "1;38;5;45") << "   "
            << paint("quit", "1;38;5;203") << "\n\n";
}

void home(bool show_banner = false) {
  if (show_banner) banner();
  else std::cout << "\n" << paint("  SLAM BENCH  /  RUNNER  ·  v1.0", "1;38;5;214") << "\n";
  help();
}
}  // namespace

std::string paint(const std::string& text, const char* code) {
  return colorEnabled() ? std::string("\033[") + code + "m" + text + "\033[0m" : text;
}

Console::Console(Actions actions) : actions_(std::move(actions)) {}

int Console::run() {
  struct sigaction action{};
  action.sa_handler = handleInterrupt;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;  // Let poll() return so Ctrl-C can be handled by this console.
  sigaction(SIGINT, &action, nullptr);

  home(true);
  std::string line;
  bool saw_active_run = false;
  while (true) {
    prompt();
    std::string async_error;
    const auto result = readLine(line,"bench> ",true,[&] {
      if (!actions_.take_error) return false;
      async_error=actions_.take_error();
      if (!async_error.empty()) return true;
      if (saw_active_run && !actions_.is_running()) return true;
      if (actions_.is_running()) saw_active_run = true;
      return false;
    });
    if (result == InputResult::Interrupt) {
      if (confirmExit(actions_.is_running())) return 0;
      continue;
    }
    if (result == InputResult::RunError) {
      if (async_error.empty()) {
        saw_active_run = false;
        std::cout << "\r\033[2K";
        home();
        continue;
      }
      std::cout << "\r\033[2K\n" << paint(std::string("  Error: ")+async_error, "1;38;5;203") << "\n\n";
      saw_active_run = false;
      home();
      continue;
    }
    if (result == InputResult::EndOfInput) return 0;

    std::istringstream input(line);
    std::string command;
    input >> command;
    try {
      if (command.empty()) continue;
      if (command == "test") {
        actions_.test();
        int selected = 1;
        const auto choice = chooseTestSequence(selected);
        if (choice == MenuResult::Interrupt) {
          if (confirmExit(actions_.is_running())) return 0;
          home();
        } else if (choice == MenuResult::Confirmed) {
          home();
          actions_.select(selected);
          actions_.play();
          saw_active_run = true;
          std::cout << paint(std::string("  ▶  Test queued · Sequence ") + (selected < 10 ? "0" : "") + std::to_string(selected) + " · waiting for SLAM", "1;38;5;82") << "\n\n";
        } else {
          home();
        }
      } else if (command == "eval") {
        std::string student_id;
        const auto id_choice=promptStudentId(student_id);
        if (id_choice==MenuResult::Interrupt) {
          if (confirmExit(actions_.is_running())) return 0;
          continue;
        }
        if (id_choice!=MenuResult::Confirmed) continue;
        const auto choice = confirmEvaluationStart();
        if (choice == MenuResult::Interrupt) {
          if (confirmExit(actions_.is_running())) return 0;
        } else if (choice == MenuResult::Confirmed) {
          actions_.evaluation(student_id);
          actions_.play();
          saw_active_run = true;
          std::cout << paint("  ▶  Evaluation queued · waiting for SLAM", "1;38;5;82") << "\n\n";
        }
      } else if (command == "select") {
        int id{};
        if (!(input >> id)) throw std::runtime_error("select requires a sequence number from 1 to 6");
        actions_.select(id);
        std::cout << "  Selected sequence " << id << '\n';
      } else if (command == "play") {
        actions_.play();
        saw_active_run = true;
        std::cout << paint("  ▶  Run started", "1;38;5;82") << '\n';
      } else if (command == "pause") {
        actions_.pause();
        std::cout << paint("  ‖  Playback paused", "38;5;214") << '\n';
      } else if (command == "resume") {
        actions_.resume();
        std::cout << paint("  ▶  Playback resumed", "38;5;82") << '\n';
      } else if (command == "cancel" || command == "stop") {
        actions_.cancel();
        std::cout << paint("  ×  Cancellation requested", "38;5;203") << '\n';
      } else if (command == "status") {
        actions_.status();
      } else if (command == "help" || command == "?") {
        help();
      } else if (command == "quit" || command == "exit") {
        if (confirmExit(actions_.is_running())) return 0;
      } else {
        std::cout << paint("  Unknown command. Type 'help' to see available commands.", "38;5;214") << '\n';
      }
    } catch (const std::exception& e) {
      std::cerr << paint("  Error: ", "1;38;5;203") << e.what() << '\n';
    }
  }
}
}  // namespace slam_bench::cli
