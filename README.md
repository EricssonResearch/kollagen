# kollagen

A Collaborative SLAM Pose Graph Generator

This is a header-only data generator library which generates multi-agent,
M3500esque pose-graphs with both intra-agent and inter-agent loop-closures.
It's primary use is within the Collaborative SLAM activity.

The package can be used either by interacting directly with the C++ API (see
[example.cpp](src/example.cpp) and [iSAM2example.cpp](src/iSAM2example.cpp)) or
by JSON files &ndash; using either the python pip package, or the
[generate](src/generate.cpp) binary made available after building.

## Overview of the generation procedure

In the generated multi-agent pose graphs, inspired by the popular M3500
dataset, introduced by [Olson et al. in 2006](http://rvsn.csail.mit.edu/content/eolson/graphoptim/eolson-graphoptim2006.pdf),
the agents move on a planar grid world.  The pose graph consists of nodes
and edges, where the nodes represent the poses of the agents and the edges
represent a relative transformation from one pose to the other.

The pose graph generation has three major components:
1. **Ground truth generation:**  
To generate the ground truth of each agent, we first let each agent turn
randomly by -180, -90, 0, or 90 degrees.  Then the agent moves a
pre-defined amount of steps in that direction.  This is repeated for a
user-specified amount of times.

2. **Noisy odometry generation:**  
The ground truth of two consecutive poses is used to obtain a true distance
and relative angle between the poses.  The distance and relative angle are
each modified by adding a zero-mean normally distributed noise terms with a
user-specified standard deviations.  In this way, ***kollagen*** generates
noisy odometry for all consecutive poses. The noisy odometry is encoded in
the edge between the two consecutive nodes in the form of a noisy relative
transformation.

3. **Loop closure generation:**  
Loop closures are edges between two (possibly non-consecutive) nodes. The
idea behind loop closures is that an agent recognizes a pose and creates an
edge, encoding a noisy relative transformation between the poses, in the
pose graph between its current pose and the recognized pose.  The noisy
relative transformation encoded by the edge is obtained similarly to the
noisy odometry, i.e., adding a zero-mean normally distributed noise to the
true distance and relative angle between the poses, which is then used to
obtain the noisy relative transformation.
There are two types of loop closures in ***kollagen***:
   - **Intra-agent loop closures** are edges between (possibly
     non-consecutive) nodes of the *same* agent. To produce intra-agent
     loop closures, we iterate through all poses, i.e., nodes, of an
     agent's pose graph and create a loop closure between the current and a
     previous poses within a certain distance based on a probability
     proportional to the distance between the poses.
   
   - **Inter-agent loop closures** are edges between nodes of *different*
     agents. To produce inter-agent loop closures, we iterate through all
     poses, i.e., nodes, of one agents and create a loop closure between
     the current pose of that agent and the pose of another agent within a
     certain distance based on a probability proportional to the distance
     between the poses.

## pip package

If only interested in generating datasets (be it multig2o or singleg2o) from
JSON, the provided pip package could be what you are after.

To install, enter the project root, and issue

```bash
pip3 install .
```

After this, the CLI command `kollagenr8` should become available. Issue 

```bash
kollagenr8 --help
```

for instructions on how to use.

For Windows builds, make sure to have up-to-date versions of MSVC and the
Windows SDK. See further below for tested versions.

### Troubleshooting

Make sure you have a modern, C++20 compliant, compiler (tested with `clang` 14)

## Building

### Linux

The building has been tested on Ubuntu 22.04 (Jammy) with Clang 14.

```bash
# Clone the repository
git clone https://github.com/EricssonResearch/kollagen.git

# Update aptitude repositories and install debian-package dependencies
sudo apt update
sudo apt install cmake clang

# OPTIONAL: Create python virtual environment and install python dependencies for
# plotting
sudo apt install python3-pip
pip3 install virtualenv
virtualenv -p python3 env
source env/bin/activate
pip3 install matplotlib numpy

# Create build folder
mkdir build && cd build

# Configure make-files, setting Clang 14 as compiler
export CC=clang
export CXX=clang++
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DKOLLAGEN_ENABLE_TESTS=OFF -DKOLLAGEN_ENABLE_GTSAM=OFF -DKOLLAGEN_BUILD_EXAMPLES=ON

# Compile
make -j$(nproc --all)
```

### Windows

Building has been tested with MSVC v143 with Windows 10 SDK 10.0.20348.

### macOS

Python installation with pip has been testen on macOS Monterey 12.6.2 with clang 14.0.0

## Citation

If using kollagen in your research, we would appreciate if you could cite our work:

```bibtex
@misc{kollagen,
  doi = {10.48550/ARXIV.2303.04753},
  url = {https://arxiv.org/abs/2303.04753},
  author = {Sundin, Roberto C. and Umsonst, David},
  title = {kollagen: A Collaborative SLAM Pose Graph Generator},
  publisher = {arXiv},
  year = {2023},
}
```
