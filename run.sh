#!/bin/bash

# Check if build folder exists, if not, create it
if [ ! -d "build" ]; then
  mkdir build
fi

# Move to build folder
cd build

# Configure the project
cmake ..

# Build the project
make -j4

# Run the project
./ARProject
