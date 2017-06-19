# CarND-Controls-MPC

[image1]: ./MPC_drive_sample.png "MPC drive sample image"
[image2]: ./Kinematics_model.png "Kinematics model"
[image3]: ./Kinematics_model_equation.png "Kinematics model equations"
[image4]: ./CTE.png "Cross track error"
[image5]: ./Orientation_error.png "Orientation error"

This project aims at using Model Predictive Controller (MPC) to control a vehicle to follow expected trajectory in a simulator. 

As the screenshot shows below, the expected trajetory is illustrated as yellow line connecting waypoints. The green line is the trajectory projected by MPC controller for the limited time in (N steps with delta_t in time lapse of each step) the future.

![alt_text][image1] 

---

## Model Predictive Control

MPC is a well-established technique for controlling multivariable systems subject to constraints on manipulated variables and outputs in an optimized way. It has long history of success in process industries, and recently it expanded into automotive and aerospace industries as well. A pretty good course can be find [here](http://cse.lab.imtlucca.it/~bemporad/mpc_course.html). 

Here in this project, MPC converts a control problem into an optimization problem. It uses [Ipopt](https://projects.coin-or.org/Ipopt), a large-scale nonlinear optimization library to find a solution minimizing a cost function while certain constraints apply. [CppAD](https://www.coin-or.org/CppAD/) is involved in the process to provide derivative valuse (AD: algorithmic defferentiation) required in the optmizaiton process. A simple [example](https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm) is useful to help understand how a problem is set up using CppAD and Ipopt. 

## Experiments on value selection

- Both 2nd-order and 3rd-order polynomials are tried, no big difference witnessed. To speed up calcuation, 2nd order polynomial is selected for final solution.

- Various weights were tried on different categories of cost function elements. Final choice is to only accept very small error in CTE and orientation error, and the change/delta in steering angle, as well as its change rate, should also be very very small to make this MPC controller work. (Details in code snippet below quoted)

- Various N and dt values are tried. N = 30, 20, 10, with dt 

## Vehicle Kinematics Model
Below kinematics model is used:
![alt_text][image2] 

```Latex
 x_{t+1} = x_t + v_t * cos(\psi_t)*dt \\
 y_{t+1} = y_t + v_t * sin(\psi_t)*dt \\
 \psi_{t+1} = \psi_t + v_t*\delta*dt/L_f \\
 v_{t+1} = v_t + \alpha*dt
```
![alt_text][image3] 

The state of the model includes:

- (x,y): Co-ordinates of vehicle
- psi: Vehicle orientation angle
- v: Vehicle speed
- CTE: Explained below.
- Error_psi: Orientation error. Explained below. 

Actuations:
- \delta: Steering angle actuator. Value in range [-25, 25] degrees.
- \alpha: Throttling actuator. Value in range [-1 (brake), +1 (throttle)]. 

## Input, Output and Data flow
###Input 
In every round of control parameter calculation, the simulator will first return/provide these information as input:

- Expected trajectory map waypoints: ptsx (array) and ptsy (array) in world coordinates.
- Orientation psi (float) relative to world X axis
- (x,y) vehicle coordinates in world.
- steering_angle, throttle, speed

### World -> Vehicle coordinates conversion
The world map coordinates are converted to vehicle coordinates thus the x_t = 0.0 which is very convenient for certain variables calculation, such as initial CTE and Orientation Error (below). 

Conversion from world matrix to vehicle coordinates:
```
[x,y]' = [cos(psi) sin(psi); -sin(psi) cos(psi)] * [delta_x, delta_y]'
```
Next step is polynomial fitting of map waypoints to provide an expected trajectory. I tried both 3rd order and 2nd order polynomials and I chose a 2nd-order polynomial and found it is good enough for this project. Apparently, 2nd order one should run faster due to less complexity and thus less calculation. 
```
          // 2nd order polynomial is good enough. 3rd order seems a bit more stable in curves.
          auto coeffs = polyfit(ptsx_eigen, ptsy_eigen, 2);

```

### Init state, CTE and Orientation Error calcuation
Convenient calculation of initial CTE and Orientation Error due to x_t=0:
```
          // Initial cross track error 
          //double cte = polyeval(coeffs, 0) - 0;
          double cte = coeffs[0];
          // Initial orientation error: arctan(f'(x))
          double epsi = 0.0 - atan(coeffs[1]);
```
Initial states are simple:
```
          // Transformed all into car coordinates, so x,y,psi are all zeroes for init.
          px = 0.0;
          py = 0.0;
          psi = 0.0;
          v = <whatever read from simulator>;
```

### Deal with 100ms latency
As in real-life, actuations are not happening real-time, there's a delay from issuing commands to it actually takes effect. A 0.1 second latency is simulated in this project of MPC controller. 

The trick is to let the Kinematics model run for 0.1 second and provide the ending state as "initial state" to optimizer. 
```
          // Account for 100ms delay - using kinematics model to predict the "init" state after latency
          double latency = 0.1;
          px += v*latency; // Follow current direction, thus only x coordinates will change. Y stays unchanged.
          psi -= v*steer*latency/Lf;
          cte += v*sin(epsi)*latency;
          epsi -= v*steer*latency/Lf;
          // V has to be put last as previous two need original value of v.
          v += throttle*latency;
```

### Optimizer kicks in
Finally the states and polynomial coeffs are fed into optimizer/solver. The steer_value and throttle_value are calculated and fed back to simulator. 
```
          state << px, py, psi, v, cte, epsi;

          auto tmp_vars = mpc.Solve(state, coeffs);
          double steer_value = tmp_vars[0];
          double throttle_value = tmp_vars[1];

          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;
```

Then the simulator comes back with new data and this flow goes back to starting of the loop. 

## Optimization solver
Optimization is solved under two constraints: minimizing cost function and condition constraints. 

### Cost function
There are several categories of costs accumulated as below:

- CTE (cross track error)
![alt_text][image4]
- Orientation Error
![alt_text][image5]
- Expected speed vs. current speed error
- Regularization costs, such as items to penalize too big and too abrupt changes in actuations (steering angle and throttle). 

Various experiments with different weights applying on these categories had been tried. The final setup is shown in MPC.cpp as below. We really only accept very small error in CTE and orientation error, and the change/delta in steering angle, as well as its change rate, should also be very very small to make this MPC controller work. 

```c++

    // cost based on reference state
    for (unsigned int t=0; t<N; t++) {
        fg[0] += 2000*CppAD::pow(vars[cte_start+t], 2);
        fg[0] += 2000*CppAD::pow(vars[epsi_start+t], 2);
        fg[0] += CppAD::pow(vars[v_start+t] - ref_v, 2);
      }

    // cost to penalize big actuations
    for (unsigned int t=0; t<N-1; t++) {
        fg[0] += 100*CppAD::pow(vars[delta_start + t], 2);
        fg[0] += CppAD::pow(vars[a_start + t], 2);
      }

    // cost to smooth actuation changes
    for (unsigned int t=0; t<N-2; t++) {
        fg[0] += 2000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      }

```

### Condition constraints
Variables are constrained for certain value ranges thus optimizer won't try values outside of the range. 

- State transition iteration

```
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 +
              coeffs[2] * CppAD::pow(x0, 2);
      // Jacobian
      AD<double> f0_jacobian = coeffs[1] +
              2 * coeffs[2] * x0;
      AD<double> f0_prime = CppAD::atan(f0_jacobian);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - f0_prime) + v0 * delta0 / Lf * dt);

```

- Value range constraint

```
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (unsigned int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (unsigned int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (unsigned int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

```


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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

