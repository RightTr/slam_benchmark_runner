# Vendored Pangolin

This directory contains the Pangolin v0.9.6 source tree from
https://github.com/stevenlovegrove/Pangolin at commit
`3667bb76edcc3e4111d2bee7935ce204f04026d9`.

Pangolin is licensed under the MIT terms in `LICENCE`. Its optional vcpkg and
pybind11 Git submodules are not needed for this project's C++ RGB-D viewer;
`build_ros.sh` disables the Pangolin Python module, examples, tools, and tests.
