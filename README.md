# Extended Kalman Filter Project:
Self-Driving Car Engineer Nanodegree Program

---

This Project is the sixth task (Project 1 of Term 2) of the Udacity Self-Driving Car Nanodegree program. 
The main goal of the project is to apply Extended Kalman Filter to fuse data from LIDAR and Radar sensors of a self driving car using C++.



## Content of this project:

src a directory with the project code:

main.cpp - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE

FusionEKF.cpp - initializes the filter, calls the predict function, calls the update function

kalman_filter.cpp- defines the predict function, the update function for lidar, and the update function for radar

tools.cpp - a function to calculate RMSE and the Jacobian matrix

data a directory with two input files, provided by Udacity

Docs a directory with files formats description


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

