# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Jack Zhang
4/8/18
---

This project implements a MPC (Model Predictive Control) in C++ for navigating a car around a test track in a simulator. For each timestep, the position of the car, as well as path-planning waypoints, are sent to the MPC in map coordinates. In addition, speed, angle, steering, and throttle information are also sent. The MPC uses this information to update the global kinematic model for determining the proper actuator commands (steering angle and throttle) to the vehicle.


