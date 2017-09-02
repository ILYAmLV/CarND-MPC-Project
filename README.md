# Self-Driving Car Engineer Nanodegree Program
## MPC - Project 5

<p align="center">
    <img src="./imgs/mpc.gif" width="600">
</p>

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Project Description

The goal of this project was to implement **` Model Predictive Control`** in `C++` to drive the car around a track in a simulation. Unlike the last project **` PID Control`** the simulator does not provide the **`Cross Track Error`** `(CTE)`, additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency. Our job was choosing the state, inputs, dynamics, constraints and implementing the MPC by modyfying `MPC.cpp`, `MPC.h` and `main.cpp`.

### The Model

The model used for this project is a kinematic one, it is a simplification of a dynamic model and as such it ignores tire forces, gravity, and mass. 

<p align="center">
    <img src="./imgs/model.png" width="600">
</p>
