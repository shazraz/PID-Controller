# PID Controller

## 1. Introduction
This project implements a PID Controller in C++ to generate throttle and steer values for a vehicle using the speed and crosstrack error (CTE) values passed to the code via a simulator. This project was completed as part of Term 2 of Udacity's Self-Driving Car Nanodegree program.

## 2. Project Environment
The project was built using the Ubuntu 16-04 bash shell in Windows 10. Instructions to set this up can be found [here](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/). The following dependencies need to be in place to build and execute the project.

* [Udacity SDC Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4
* uWebSocketIO (installed via the [install-ubuntu.sh](https://github.com/shazraz/Extended-Kalman-Filter/blob/master/install-ubuntu.sh) script) 

The project consists primarily of the following files located in the [src](https://github.com/shazraz/PID-Controller/tree/master/src) folder:

* [main.cpp](https://github.com/shazraz/PID-Controller/blob/master/src/main.cpp): Interfaces with the simulator using uWebSocketIO to recieve speed and CTE values to generate the steer and throttle values.
* [main.cpp](https://github.com/shazraz/PID-Controller/blob/master/src/PID.cpp): Implements the PID controller.

Once the environment is ready, the code can be tested as follows:

1. Launch the simulator and select the PID Controller project
2. Execute ```./pid Kp Ki Kd MaxSpeed``` from the build directory
3. Click Start in the simulator

## 3. Results & Discussion
The PID executable expects four arguments at run-time defined as follows:

* Kp: Coefficient for proportional error term
* Ki: Coefficient for integral error term
* Kd: Coefficient for differential error term
* MaxSpeed: Maximum speed of the vehicle in the simulator

The overall steering angle is the negative sum of the product of these coefficients with their respective error terms where each component plays a different role in the steering angle value. 

The proportional error term, ```Kp_ x p_error_```, returns a steering angle component value that is proportional to the CTE and as a result the vehicle oscillates around a reference positon in the simulator if the other terms are set to zero. In particular, the oscillations become large and the controller unstable around turns where the CTE increases. 

The differential error term, ```Kd_ x d_error_```, returns a steering angle component value equal to the difference in the CTE between the current and previous timestep. This is useful for generating large steering angles when the vehicle is significantly far from the reference position and smaller steering angle values the closer the vehicle gets to its reference position. This creates a damping effect in the oscillations seen when using only the proportional error term when used correctly.

The integral error term, ```Ki_ x i_error_```, 
