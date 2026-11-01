cmake_minimum_required(VERSION 3.8)
project(slam_bench_runner)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(slam_bench_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(OpenSSL REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(SODIUM_LIBRARY sodium REQUIRED)
find_library(ZSTD_LIBRARY zstd REQUIRED)

# Teacher builds embed the dataset decryption key in the private core library.
# Both generated header copies are teacher-only and are never installed.
set(SLAM_BENCH_DATASET_KEY_FILE
  "${CMAKE_CURRENT_SOURCE_DIR}/../key/dataset_aes256.key" CACHE FILEPATH
  "Teacher-only raw AES-256 dataset key")
if(NOT EXISTS "${SLAM_BENCH_DATASET_KEY_FILE}")
  message(FATAL_ERROR "Dataset key missing for core build: ${SLAM_BENCH_DATASET_KEY_FILE}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${SLAM_BENCH_DATASET_KEY_FILE}")
file(READ "${SLAM_BENCH_DATASET_KEY_FILE}" DATASET_KEY_HEX HEX)
string(LENGTH "${DATASET_KEY_HEX}" DATASET_KEY_HEX_LENGTH)
if(NOT DATASET_KEY_HEX_LENGTH EQUAL 64)
  message(FATAL_ERROR "Dataset key must contain exactly 32 raw bytes")
endif()
string(REGEX MATCHALL "[0-9a-f][0-9a-f]" DATASET_KEY_BYTES "${DATASET_KEY_HEX}")
set(DATASET_KEY_INITIALIZER "")
foreach(BYTE IN LISTS DATASET_KEY_BYTES)
  string(APPEND DATASET_KEY_INITIALIZER "0x${BYTE},")
endforeach()
set(EMBEDDED_DATASET_KEY_HEADER
  "#pragma once\n#include <array>\nnamespace slam_bench::private_data {\ninline std::array<unsigned char, 32> datasetKey() { return {${DATASET_KEY_INITIALIZER}}; }\n}\n")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/embedded_dataset_key.hpp"
  "${EMBEDDED_DATASET_KEY_HEADER}")
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/src/embedded_dataset_key.hpp"
  "${EMBEDDED_DATASET_KEY_HEADER}")
unset(DATASET_KEY_HEX)
unset(DATASET_KEY_BYTES)
unset(DATASET_KEY_INITIALIZER)
unset(EMBEDDED_DATASET_KEY_HEADER)

add_library(slam_bench_core SHARED
  src/benchmark_core.cpp src/benchmark_input.cpp src/benchmark_output.cpp
  src/generated_groundtruth.cpp)
set_target_properties(slam_bench_core PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib")
target_compile_definitions(slam_bench_core PRIVATE SLAM_BENCH_CORE_BUILD)
target_compile_options(slam_bench_core PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
target_include_directories(slam_bench_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(slam_bench_core ${OpenCV_LIBRARIES} OpenSSL::Crypto Eigen3::Eigen ${SODIUM_LIBRARY} ${ZSTD_LIBRARY})
add_library(slam_bench_cli SHARED src/cli.cpp)
set_target_properties(slam_bench_cli PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib")
target_include_directories(slam_bench_cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_executable(slam_bench src/slam_bench.cpp src/benchmark_runner.cpp)
ament_target_dependencies(slam_bench rclcpp sensor_msgs nav_msgs slam_bench_msgs)
target_include_directories(slam_bench PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(slam_bench ${OpenCV_LIBRARIES} slam_bench_core slam_bench_cli)
add_executable(slam_bench_teacher teacher/report_tool.cpp)
target_link_libraries(slam_bench_teacher OpenSSL::Crypto ${SODIUM_LIBRARY} ${ZSTD_LIBRARY})
install(TARGETS slam_bench DESTINATION lib/${PROJECT_NAME})
install(TARGETS slam_bench_core LIBRARY DESTINATION lib)
install(TARGETS slam_bench_cli LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_package()
