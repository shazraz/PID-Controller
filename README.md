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
* [PID.cpp](https://github.com/shazraz/PID-Controller/blob/master/src/PID.cpp): Implements the PID controller.

Once the environment is ready, the code can be tested as follows:

1. Launch the simulator and select the PID Controller project
2. Execute ```./pid Kp Ki Kd MaxSpeed``` from the build directory
3. Click Start in the simulator

## 3. Tuning Approach
The PID executable expects four arguments at run-time defined as follows:

* Kp: Coefficient for proportional error term
* Ki: Coefficient for integral error term
* Kd: Coefficient for differential error term
* MaxSpeed: Maximum speed of the vehicle in the simulator
* [OPTIONAL] Outputfile: Data file to output CTE, steering value and individual error terms

The overall steering angle generated by the steering PID controller is the negative sum of the product of these coefficients with their respective error terms where each component plays a different role in the steering angle value. 

The proportional error term, ```Kp_ x p_error_```, returns a steering angle component value that is proportional to the CTE and as a result the vehicle oscillates around a reference positon in the simulator if the other terms are set to zero. In particular, the oscillations become large and the controller unstable around turns where the CTE increases. 

The differential error term, ```Kd_ x d_error_```, returns a steering angle component value equal to the difference in the CTE between the current and previous timestep. This is useful for generating large steering angles when the vehicle is significantly far from the reference position and smaller steering angle values the closer the vehicle gets to its reference position. This creates a damping effect in the oscillations seen when using only the proportional error term when used correctly.

The integral error term, ```Ki_ x i_error_```, returns a steering angle component value that is equal to the cumulative error accrued by the PID controller since initializing. This is useful for correcting biases that may exist in the calibration of the vehicle steering.

It is important to note that each of these components has an additive effect on the steering angle, i.e. the addition of components increases the overall magnitude of the steering angle for a given CTE. The objective is therefore to come up with values for the coefficients that would generate very small steering angle values for small CTEs and decreasing steering angle values as CTE decreases. This can be achieved by tuning the controller so that the predominant contributor to the steering angle is the differential error term when navigating straight sections and the proportional error term when navigating curves. This results in a reduced oscillations in the straight sections of the track and also allows for sharp manuevering around curves. This can be done by setting Kd to be an order of magnitude larger than Kp. The values of the coefficients are then manually adjusted to successfully navigate the vehicle around the track.

The generated steering angle values are then bound to the interval [-1,1] which corresponds to the minimum and maximum steering angles of [-25 deg, 25 deg] in the simulator.

## 4. Implementation notes

A separate PID controller is used to control the throttle value of the vehicle to maintain speed. The coefficient values of this controller are set separately and hard-coded in the compiled executable.

The target speed of the vehicle is calculated as follows:

```Target Speed = Max Speed - |Steering Angle| x Speed Reduction```

where the maximum speed is defined at runtime and Speed Reduction is a tunable parameter. The CTE used for the speed controller is then the difference between the current speed and the target speed. This enables the vehicle to slow down during sharp turns and accelarate during straight stretches of the roadway.

## 5. Results
The speed controller was initialized at {Kp, Ki, Kd} = {0.2, 0, 0.1} which did a reasonable job of maintaining speed so the parameters weren't tuned further. The maximum speed was initially set to 30 with a speed reduction of 20.

The initial choice of parameters, {Kp, Ki, Kd}, for the steering controller was {0.5, 0, 0} which resulted in large oscillations of the vehicle as expected. This was reduced to {0.1, 0, 0} which reduced the magnitude of steering angles initially but the oscillations remained. A differential term was then introduced giving a parameter set of {0.1, 0, 1.0} which was able to successfully navigate the track without any issues.

The speed was then increased in increments of 10mph resulting in oscillations becoming more visible at higher speeds. The proportional term was therefore slightly reduced and the differential term increased resulting in a parameter set of {0.09, 0, 1.2} at a speed of 60mph which resulted in satisfactory performance. The speed reduction was set to 45. 

Finally a small integral term was introduced to account for any biases that may exist which had a very subtle effect on the vehicle performance. In particular, mild overshoots were observed at certain turns. 

The final choice of parameters, after additional tuning, is as follows:

* Speed Controller: {0.2, 0, 0.1}
* Steer Controller: {0.095, 0.0001, 1.7}
* Max Speed: 60 mph
* Speed Reduction: 55 mph

The performance can be verified by executing ```./pid 0.095 0.0001 1.7 60``` and then running the simulator.

*NOTE: The simulator was set to Fastest graphics mode and a window size of 800x600 during the tuning process. It was observed that higher graphics settings significantly deteriorated the performance of the controller due to limitations in the hardware used to run the simulator.*

The plots below compare the contributions of the various error terms for various sections of the track.

One Lap:

<img src="./graphics/full_track.png" width="750"> 

Initial straight section:

<img src="./graphics/straight.png" width="750"> 

Turns after bridge:

<img src="./graphics/turns.png" width="750"> 

As expected, the proportional term and differential terms are minimal during the straight sections of the roadway whereas the proportional term dominates in the curved sections. The integral term plays a minimal role in the steering angle confirming the absence of any significant bias in the vehicle steering.


