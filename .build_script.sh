#!/bin/bash
# .build_script.sh

set -e

source ./.build_helper.sh

generate_data_download_debian_packages
generate_data_python_environment
generate_data_python_dependencies
generate_data_prepare_build
generate_data_cmake
generate_data_build
generate_data_run_tests
generate_data_plot_and_animate
generate_data_build_docs