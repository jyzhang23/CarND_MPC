# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

This project implements a MPC (Model Predictive Control) in C++ for navigating a car around a test track in a simulator. For each timestep, the position of the car, as well as path-planning waypoints, are sent to the MPC in map coordinates. In addition, speed, angle, steering, and throttle information are also sent. The MPC uses this information to update the global kinematic model for determining the proper actuator commands (steering angle and throttle) to the vehicle.

## The Model

The global kinematic model contains 4 vehicle states (x position, y position, heading angle, and velocity) referenced to the global map coordinates, as well as 2 error states (cross track error and heading error). The kinematic equations for determining the next state, given a time step of *dt* are:

1. *x(t+1) = x(t) + v(t) * cos(psi(t)) * dt*
2. *y(t+1) = y(t) + v(t) * sin(psi(t)) * dt*
3. *psi(t+1) = psi(t) + v(t) / Lf * delta * dt*
4. *v(t+1) = v(t) + a(t) * dt*

where *x* and *y* are positions, *psi* is the heading angle with respect to the x-axis, *v* is the velocity, *Lf* is the measurement between the front of the vehicle and its center of gravity, *delta* is the steering ange, *a* is the acceleration, and *dt* is the time step.

The error states are determined by:

5. *cte(t+1) = cte(t) + v(t) * sin(epsi(t)) * dt*
6. *epsi(t+1) = epsi(t) + v(t) / Lf * delta(t)* dt*

where *cte* is the cross track error and *epsi* is the heading error. 

The path planning waypoints are fit to a polynomial function *f*, which allows us to calculate *cte(t)* and *epsi(t)* as:

7. *cte(t)* = *y(t)* - *f(x(t))*
8. *epsi(t)* = *psi(t)* - *arctan(f'(x(t)))*

At every time *t*, the vehicle state *[x y psi v cte epsi]* are passed to the MPC class, along with coefficients for a 2nd order polynomial. This polynomial is used to calculate the reference trajectory. The state of the next *N* timesteps are calculated based on equations 1-8 in the *FG_eval* class. 

The actuator values are calculated based on a cost function, which attemps to minimize *cte* and *epsi*. In addition, additional terms are added to the cost function to minimize the use of the actuators as well as the gap between sequential actuations, in order to make the steering and braking more smooth. A final terms is added to the cost function to minimize the difference between the car speed and a reference speed, which I selected as 60mph. 

The prefactors to the 7 different cost function terms were selected by trial and error. Since minimizing the error terms were the primary focus, these prefactors were set to 1000. Smoothing the actuactions were also important, but less so than minimizing the errors, so the prefactors for those were set to 10, with 100 set for smoothing the steering actuation. The prefactor for the speed was the lowest, which was set to 1.

## Timesteps

The number of timesteps *N* and the time between each timestep *dt* were chosen so that the total time the MPC models (*N * dt*) is approximately the same length as the waypoints. This corresponded to ~1s (*N* = 10, *dt* = .1). I also tried varying *dt* to 0.05 or 0.2, and changing *N* to 20 or 5. In both cases, the MPC model did not work well and the car swerved quite a lot on the track. Using *dt* = .1 worked best, perhaps due to matching it with the latency.

## Polynomial Fitting & MPC Prepocessing

A polynomial function was fit to the waypoints and the coefficients were passed as inputs to the MPC class. A polynomial degree of 2 was chosen, since the 6 waypoints for every timestep corresponded to approximately a parabola shape. 

The waypoints were preprocessed to transform them from map coordinates to vehicle coordinates. This involves a translation and rotation, so that the transformed *x*, *y*, and *psi*, states are all zero. This was achieved by the following transformation:

9. *x_t = (x_w - x_c) * cos(-psi) - (y_w - y_c) * sin(-psi)*
10. *y_t = (x_w - x_c) * sin(-psi) + (y_w - y_c) * cos(-psi)*

where *_t* indicates the transformed coordinate, and *_w*, *_c* indicates the waypoint and car coordinate in map space.

## MPC with Latency

In order to account for a 100ms latency, the transformed coordinates are projected into the future by 100ms. This is done by using the same equations as 1-8 to calculate the vehicle position at time *t=.1* instead of *t=0*. This new state is passed to the MPC class to determine the actuation values of the vehicle.
