generate_data_debian_dependencies() {
    echo "cmake libgtest-dev libgmock-dev libgflags-dev python3-pip clang-14 ffmpeg doxygen"
}

generate_data_download_debian_packages() {
  apt update && apt -y install $(generate_data_debian_dependencies)
}

generate_data_download_debian_packages_sudo() {
  sudo apt update && sudo apt -y install $(generate_data_debian_dependencies)
}

generate_data_python_environment() {
  echo "Creating python virtual environment and installing requirements"
  pip3 install virtualenv
  virtualenv -p python3 env $1
  source env/bin/activate
}

generate_data_python_dependencies() {
  pip3 install matplotlib numpy
}

generate_data_prepare_build() {
  mkdir "$1"build
  cd "$1"build
}

generate_data_cmake() {
  CC=clang-14 CXX=clang++-14 cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX="./install" -DDATAGEN_ENABLE_TESTS=ON
}

generate_data_run_tests() {
  echo "Run tests..."
  ctest -VV --output-on-failure
  echo "Run tests completed."
}

generate_data_build() {
  echo "Move to build stage"
  echo "Compiling using all $(nproc --all) available cores"
  make -j$(nproc --all)
  echo "Compiling done"
}

generate_data_run_example() {
  echo "Running example"
  ./example
}

generate_data_build_docs() {
  echo "Building docs"
  cd ..
  doxygen Doxyfile
}

generate_data_plot_and_animate() {
  generate_data_run_example
  echo "Animating"
  ../python/animate.py example
  echo "Plotting"
  ../python/plot.py example
}
