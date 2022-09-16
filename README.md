# kollagen

A Collaborative SLAM Pose Graph Generator

This is a header-only data generator library which generates multi-agent,
M3500esque pose-graphs with both intra-agent and inter-agent loop-closures.

For an example on how to use, see [example.cpp](src/example.cpp).

## Building

The building is tested on Ubuntu 22.04 (Jammy) with Clang 14.

```bash
# Clone the repository
git clone https://github.com/EricssonResearch/kollagen.git

# Update aptitude repositories and install debian-package dependencies
sudo apt update
sudo apt install cmake libgtest-dev libgmock-dev libgflags-dev python3-pip clang-14 ffmpeg doxygen

# Create python virtual environment (optional)
pip3 install virtualenv
virtualenv -p python3 env
source env/bin/activate

# Install python dependencies for plotting
pip3 install matplotlib numpy

# Create build folder
mkdir build && cd build

# Configure make-files, setting Clang 14 as compiler
CC=clang-14 CXX=clang++-14 cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX="./install" -DDATAGEN_ENABLE_TESTS=OFF -DDATAGEN_ENABLE_GTSAM=OFF

# Compile
make -j$(nproc --all)
```

To try out the package, run [example.cpp](src/example.cpp) and plot the
resulting pose graphs:

```bash
./example
../python/plot.py example
```
