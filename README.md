# PID Controller

## Introduction:

Given a driving simulator which provides the cross track error (CTE) and the velocity (mph), the goal of this project is to develop a PID controller in c++ that successfully drives the vehicle around the track. In order to navigate the car around the track. I have implemented 2 PID controllers, one for steering angle control and the other for speed control.

## Tuning:

The most important part of the project is to tune the hyperparameters. This can be done by different methods suck as manual tuning, Zieglor-Nichols tuning, SGD, Twiddle. I have used manual tuning. Manual tuning is hard but, the process of tuning help us better understand every single effect of PID parameters. The following table summerizes the effect of each parameter on the system.


| Parameters   | Rise Time     | Overshoot  | Settling Time  | Steadystate error  |
| -------------|:-------------:| ----------:| --------------:| ------------------:|
| Kp           | Decrease      | Increase   | Small change   | Decrease           |
| Kd           | Small change  | Decrease   | Decrease       | No Change          |
| Ki           | Decrease      | Increase   | Small change   | Decrease           |


### Proportional (P):

This parameter controls the error proportionally. Increasing the proportional gain has the effect of proportionally increasing the control signal for the same level of error. Setting only P control is agressive and has oscillations with frequent overshoots.

### Derivative (D):

This parameter controls the rate of change of error. Addition of this term reduces the oscillary effect on the system. With devicative control, the control signal can become large of the error begins sloping upward, even while the magnitude of the error is still relatively small. This snticipation tends to add damping to the system, thereby decreasing overshoot.

The following approach is a best to tune manually,

* Set all gains to zero.
* Increase the P gain until the response to a disturbance is steady oscillation.
* Increase the D gain until the oscillations go away.
* Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
* Set P and D to the last stable values.
* Increase the gain until it brings you to a set points with the number of oscillations desired.

### Integral (I):

This parameter controls the accumulating error. Addition of this term reduces the steady state error. If there is a bias in the system, the integrator builds and builds, thereby increasing the control signal and driving the error down.

I started initially with (0.1, 0.03, 2.0) for steering control and (0.1, 0.0, 0.0) for speed control. The final parameters for my controllers are:

| Parameters   | Steering  | Speed   |
| -------------|:---------:| -------:|
| Kp           | 0.13      | 0.100   |
| Kd           | 1.00      | 0.000   |
| Ki           | 0.00      | 0.002   |

## Conclusion:

This implementation was able to achieve a maximum speed of around 40 mph without any lane violation by the ego vehicle on the given track.

As I have used a Manual tuning instead of trying twiddle method for tuning hyperparameters, My next goal would be to try twiddle method and observe the changes in hyperparameters.

<img src="Results/final_result.gif">

---

## Basic Build Instructions:

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Code Style:

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

