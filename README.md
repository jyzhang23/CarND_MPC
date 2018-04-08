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

## Timesteps


## Polynomial Fitting & MPC Prepocessing

## MPC with Latency
