# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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


## Project hint
1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.


## Report

## Model of the vehicle


State
The state for the model is [X Y PSI V]. V is the speed of car, PSI is the angle of the car to the angle of x-axis, X,Y are the 2D coordinate of car.

Actuators and update equation

Actuators delta(steering angle) and a(throttle) are modeled as the angular acceleration and acceleration. However, the real angular acceleration should be double d_psi = v / Lf * delta_steering_angle * dt

Errors CTE(cross track error) 
E-Psi(psi error) :orietation error

## Update Equation 
The code is in 121~128 for main.cpp

          fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
          fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
          fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
          fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
          fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
          fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);




## 100 millisecond latency

In the Main.cpp , I appiled the 100 millisecond in the line 102 in main.cpp file.
The latecy would affect the x,y vehicle velocity and the psi for the vihicle.
The code is as follow

            px=px+v*cos(psi)*latency;
            py=py+v*sin(psi)*latency;
            psi=psi-v/Lf*angle_steer*latency;
            v=v+pedal_throttle*latency;
            

            
The px,py would affect the polynomial fitting and mpc preprocessing

## Polynomial fitting and MPC preprocessing
I have the coordinate transform in line 115 in main.cpp. The code is as flow

                double x_shift = ptsx[i] - px;
                double y_shift = ptsy[i] - py;
                xvals[i] = x_shift * cos(0-psi) - y_shift * sin(0-psi);
                yvals[i] = x_shift * sin(0-psi) + y_shift * cos(0-psi);


## N and dt

The Time step length(N) and the Elapsed duration(dt)

N * dt = T (Predict horizon)

In theory, we would like to have a very large T. Also, we would want the dt to be as small as possible.
However, we only feet the track by 6 points with 3 degree polynomial. Besides, the enviroment change a lot in 1~2 seconds so that we should choose a T which make sense to our car. What is more, the dt would be correspond with the calculation time.The N would increase if we would likt to have a small dt.This would enhance the computation time. 
I start with the dt=0.01 and N =10. This mean my predict horizon is 0.1. 
Af first it did not perform well even the car would drive out of the road.I increase the dt from 0.01->0.04 and it work good.
I change the N from 10-> 15 and it still work well.
By the method of try and error, I finally choose dt =0.04 and N =15 for this project. 






