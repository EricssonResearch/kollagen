# kollagen

A Collaborative SLAM Pose Graph Generator

This is a header-only data generator library which generates multi-agent pose graphs.

⚠️ __NB:__ Some of the pronounced features (such as generation from json files) are currently under development

## Overview of the generation procedure

In the generated multi-agent pose graphs, inspired by the popular M3500 dataset, introduced by [Olson et al. in 2006](http://rvsn.csail.mit.edu/content/eolson/graphoptim/eolson-graphoptim2006.pdf), the agents move on a planar grid world.
The pose graph consists of nodes and edges, where the nodes represent the poses of the agents and the edges represent a relative transformation from one pose to the other.

The pose graph generation has three major components:
1. **Ground truth generation:**  
To generate the ground truth of each agent, we first let each agent turn randomly by -180, -90, 0, or 90 degrees.
Then the agent moves a pre-defined amount of steps in that direction. 
This is repeated for a user-specified amount of times.

2. **Noisy odometry generation:**  
The ground truth of two consecutive poses is used to obtain a true distance and relative angle between the poses. 
The distance and relative angle are each modified by adding a zero-mean normally distributed noise terms with a user-specified standard deviations. 
In this way, ***kollagen*** generates noisy odometry for all consecutive poses. The noisy odometry is encoded in the edge between the two consecutive nodes in the form of a noisy relative transformation.

3. **Loop closure generation:**  
Loop closures are edges between two (possibly non-consecutive) nodes. The idea behind loop closures is that an agent recognizes a pose and creates an edge, encoding a noisy relative transformation between the poses, in the pose graph between its current pose and the recognized pose. 
The noisy relative transformation encoded by the edge is obtained similarly to the noisy odometry, i.e., adding a zero-mean normally distributed noise to the true distance and relative angle between the poses, which is then used to obtain the noisy relative transformation.
There are two types of loop closures in ***kollagen***:  

   - **Intra-agent loop closures** are edges between (possibly non-consecutive) nodes of the *same* agent. To produce intra-agent loop closures, we iterate through all poses, i.e., nodes, of an agent's pose graph and create a loop closure between the current and a previous poses within a certain distance based on a probability proportional to the distance between the poses.
   
   - **Inter-agent loop closures** are edges between nodes of *different* agents. To produce inter-agent loop closures, we iterate through all poses, i.e., nodes, of one agents and create a loop closure between the current pose of that agent and the pose of another agent within a certain distance based on a probability proportional to the distance between the poses. 

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
