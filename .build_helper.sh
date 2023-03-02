kollagen_debian_dependencies() {
    echo "cmake libgtest-dev libgmock-dev libgflags-dev python3-pip clang-14 ffmpeg doxygen"
}

kollagen_download_debian_packages() {
  apt update && apt -y install $(kollagen_debian_dependencies)
}

kollagen_download_debian_packages_sudo() {
  sudo apt update && sudo apt -y install $(kollagen_debian_dependencies)
}

kollagen_python_environment() {
  echo "Creating python virtual environment and installing requirements"
  pip3 install virtualenv
  virtualenv -p python3 env $1
  source env/bin/activate
}

kollagen_python_dependencies() {
  pip3 install matplotlib numpy
}

kollagen_prepare_build() {
  mkdir build
}

kollagen_cmake() {
  CC=clang-14 CXX=clang++-14 cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX="./install" -DKOLLAGEN_ENABLE_TESTS=ON
}

kollagen_run_tests() {
  echo "Run tests..."
  ctest -VV --output-on-failure
  echo "Run tests completed."
}

kollagen_build() {
  echo "Move to build stage"
  echo "Compiling using all $(nproc --all) available cores"
  make -j$(nproc --all)
  echo "Compiling done"
}

kollagen_run_example() {
  echo "Running example"
  ./example
}

kollagen_build_docs() {
  echo "Building docs"
  doxygen Doxyfile
}

kollagen_plot_and_animate() {
  # Requires running kollagen_run_example
  echo "Animating"
  ../python/animate.py example
  echo "Plotting"
  ../python/plot.py example
}

kollagen_build_pip_package() {
  pip3 install .
}
