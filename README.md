# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
---
## Description

The Model Predictive Control (MPC) consists in generating a control signal derived from an optimized trajectory of the object movement for a number of steps in time. In this project, the task is to implement MPC to drive the car around the track. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.


---
## Rubric Points

1. The Model

- Describe the model in detail. This includes the state, actuators and update equations.

The state of the vehicle model is described with positon `x`, position `y`, orientation angle `psi` and velocity `v`, as well as the cross track error `cte` and psi error `epsi` are attached to. Controlling the vehicle we use the actuators outputs steering angle `delta` and throttle/brake value `a`. The model combines the state and actuators from the previous timestep to calculate the state for the current timestep.

The vehicle's kinematic model equations:

x[t+1]   = x[t] + v * cos(psi) * dt,
y[t+1]   = y[t] + v * sin(psi) * dt,
psi[t+1] = psi[t] + v/Lf * delta[t] * dt,
v[t+1]   = v[t] + a[t] * dt,
cte[t+1] = f(x[t]) - y[t] + v * sin(epsi[t]) * dt,
epsi[t+1] = psi[t] - psides[t] + v/Lf * delta[t] * dt.

where Lf is the length from front to centor of gravity of the vehicle.

MPC cost function consists:

- cte: cross track error
- epsi: angle error
- v - ref_v: velocity error
- delta: steering angle value
- a: throttle/brake value
- delta[t+1] - delta[t]:gap between consequent steering value
- a[t+1] - a[t]: gap between consequent throttle/brake value

The limitation of the actuators: throttle/brake is limited by interval [-1, 1], the steering is limited by interval [-0.436332, 0.436332] (-25deg - 25deg).

It is also a big topic how to tuning the weights of each cost. After tuning a number of times I got a proper set fo cost weights to let the vehicle drive smoothly without much of steering vibration.

Weight parameters for each cost:
- weight_cte = 1;
- weight_epsi = 50;
- weight_refv = 1;
- weight_delta = 100;
- weight_a = 10;
- weight_delta_gap = 500;
- weight_a_gap = 50;


2. Timestep Length and Elapsed Duration (N & dt)

- Discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. 

The final values chosen for `N` and `dt` are `10` and `0.1s`.

In this project, I set reference velocity with a relative safe value of 40 mph.

First of all, I tried N from 25 and dt from 0.05. It brings disturbance in such fast and many points calculating. It is not neccessary to predict so far and so quickly. As investigate in the waypoints, the average distance of each sequent waypoint is about 12m. In the speed of 40mph, the interval time from two waypoints will reach 0.25s. So 0.1 second will be enough for this project.

3. Polynomial Fitting and MPC Preprocessing

- A polynomial is fitted to waypoints. If waypoints are preprocessed, the vehicle state, and/or actuators prior to the MPC procedure it is described.

A third order polynomial `f(x) = a3 * pow(x,3) + a2 * pow(x,2) + a1 * x + a0` is fiited to the waypoints, since third order polynomials will fit trajectories for most roads.

The waypoints are preprocessed by transforming them to the vehicle's coordination. It helps to simplfy the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin point (0, 0) and the orientation angle psi is also 0.

So the equations of error simplify to:
cte[t] = a0,
epsi[t] = arctan(a1)

4. Model Predictive Control with Latency

- Implement Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

The original kinematic equations depend on the actuators outputs from the previous timestep. After considering a latency, the actuators outputs are applied a number of timestep `latency_time / dt` later.

``` C++
      // Consider actuator latency
      if(t > latency){
        delta0 = vars[delta_start + t - 1 -latency];
        a0 = vars[a_start + t - 1 - latency];
      }
```


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
