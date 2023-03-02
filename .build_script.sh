#!/bin/bash
# .build_script.sh

set -e

ROOT_DIR=`pwd`

source ./.build_helper.sh

kollagen_download_debian_packages
kollagen_python_environment
kollagen_python_dependencies
cd "$ROOT_DIR"
kollagen_prepare_build
cd build
kollagen_cmake
kollagen_build
kollagen_run_tests
kollagen_run_example
#kollagen_plot_and_animate
cd "$ROOT_DIR"
kollagen_build_pip_package
kollagen_build_docs
