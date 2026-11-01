#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
jobs="${BUILD_JOBS:-4}"

if [[ ! -f "$repo_root/Vocabulary/ORBvoc.txt" ]]; then
  tar -xf "$repo_root/Vocabulary/ORBvoc.txt.tar.gz" -C "$repo_root/Vocabulary"
fi

mkdir -p "$repo_root/build"
cd "$repo_root/build"
cmake "$repo_root" -DCMAKE_BUILD_TYPE=Release
cmake --build . -- -j"$jobs"
