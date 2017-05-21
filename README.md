# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Implementation Details

In this project we use a Model Predictive Control to drive a car around a track in Udacity provided
Simulator. A video of final outcome is given [here](run_60mph.mp4) or [here](https://youtu.be/E5w7pAwYNqY)

Car modelled using a kinematic model as explained in the lectures.

#### The Model:

state: [x, y, psi, v, cte, epsi]  
where:  
cte = cross track error  
epsi = error in psi

actuator: [delta, a]  
where:  
delta = steering angle  
a = throttle value

The state equations are given as  
`x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt`  
`y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt`  
`psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt`  
`v_[t+1] = v[t] + a[t] * dt`  
`cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt`  
`epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt`  

Simulator provides these values at time instance t. Simulator also provides the middle of road as a list of 6 points(px,py) in world coordinate frame. We have to use a MPC control to predict the new `delta` and `a` and pass it back to simulator to drive the car.

The pipleline I follow is:  
a) Convert trajectory points to car frame by doing a shift followed by rotation.  
b) Fit a degree-3 polynomial to find the coefficients.  
c) convert the state to future state after 100 milliseconds to account for the latency.  
d) pass along the state (with latency factored in) and car-coord trajectory coefficients to MPC solver to solve.  
e) Get back the projected trajectory alongwith actuator values (i.e. delta-steering angle and a - throttle value).  
f) Pass the actuator values and projeccted trajectory to simulator which paints the projected trajectory in green color and also uses the actutator values to control the car movement.   
g) Wait for arriaval of next telemetary event from simulator and then start the process all over again from a)   

#### MPC Explanation

* As explained in lectures, I first coded the MPC cost function which the solver minimizes. First squared error of cte, epsi and the differnce of current speed vs desired speed (60 mph in my case) is added.

* Following the explanations in lectures, the sqaure of actuator values (delta, a) is added. Also added is the squared difference between two subsequent timestamp actuator values to ensure that sudden and drastic changes get punished. For steering change, I use a multipler of 1000 as given below:  
`fg[0] += 1000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);`  
First I had used a multiple of 100 then 500, finally settled for 1000 to severly punish sudden large steering angle changes.

* I then add the constraints as explained in lectures and the mpc quiz, keeping in mind that mpc quiz was using a degree-1 polynomial while for the project, we were required to use degree-3 polynomial.

* MPC has two parameters, N = number of timestamps in future and dt = time elapse in seconds between two actuations. First I started with the value of N = 25, and dt = 0.05 just as the mpc quiz. However, it did not workout (could have been due to other bugs in code). So I redcued reference speed to 30 mph and increased N to 50 to get a longer smoother line. At slow speed it gave good results. However as I started increasing speed, N=50 at high speeds started having problem fitting a smooth degree-3 polynomial especially around curves. I also felt that 0.05 is too small for the car and threafter kept dt around 0.1 while reducing N. At a reference speed of 50, N=10 with dt=0.1 performed well. However, in order to push the speed to 60 mph, i found N=7 and dt=0.09 as acceptable.  

#### Telemetary data pre-processing

* Telemetary data provided by emulator are in world coordiates. In order to show the trajectory points on simulator, all the trajectory points are converted from world to car-frame coordinates. The code lines doing that in `main.cpp` are:  
  ```
// convert to car coordiate system
Eigen::VectorXd car_ptsx = Eigen::VectorXd(ptsx.size());
Eigen::VectorXd car_ptsy = Eigen::VectorXd(ptsx.size());
double x, y;
for(int i = 0; i < ptsx.size(); i++) {
  x = ptsx[i] - px;
  y = ptsy[i] - py;
  car_ptsx(i) = x * cos(psi) + y * sin(psi);
  car_ptsy(i) = -x * sin(psi) + y * cos(psi);
}```

* These data points are then fitted using a degree-3 polynomial. The coefficients of the fitted polynomial are one of the inputs to MPC solver.

* Latency: There's a 100 millisecond latency between actuations commands. So in order to account for it, I take the current state from telemetary data and project the current state in car-coordinates to a future timestamp using the kinematic model as expalined above in **Model** section above. In car-cordinates current state has: px = 0, py=0, psi =0 and v= data from telemetary.  

  The equations for new state with latency are:  

    ```
    double lat_x = 0.0+v*0.1;
    double lat_y = 0.0;
    double lat_psi = 0.0;
    double lat_cte = polyeval(coeffs, 0) - 0;
    double lat_epsi = lat_psi -atan(coeffs[1] + 2 * coeffs[2] * lat_x + 3 * coeffs[3] * lat_x * lat_x);
    ```  
    I had also tried to modify  `lat_psi ` using below equations but it made outcome worse.  
    ```
    double lat_y = 0.0;
    // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    double lat_psi = v/Lf*delta*0.1;
    double lat_cte = polyeval(coeffs, lat_x) - lat_y;
    ```
* The modified state along with coefficients of map trajectory is then fed into MPC solver which returns the actuator values and projected trajectory points which are passed along to emulator.




---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
