# CarND-Term2-P5-ModelPredictiveControl
Self-Driving Car Engineer Nanodegree Program: Term 2 Project 5

## The Model
A kinematic model was used in this project. The state vector consisted of the vehicle’s x and y positions, yaw angle (psi), velocity (v), cross track error (cte) and yaw error (epsi). The state vector was updated using the previous state vector and certain known parameters about the vehicle every time step ([MPC.cpp](https://github.com/nvphadnis/CarND-Term2-P5-ModelPredictiveControl/blob/master/src/MPC.cpp) lines 131-136).
 
![equations_image](/readme_images/equations_image.jpg)

## Timestep Length and Elapsed Duration (N & dt)

These values coupled with the cost function weights provided the best result at the chosen reference velocity of 60 km/hr ([MPC.cpp](https://github.com/nvphadnis/CarND-Term2-P5-ModelPredictiveControl/blob/master/src/MPC.cpp) lines 9-10).
- N = 10
- dt = 0.1 sec
- Total prediction path time (N x dt) = 1 sec

For dt = 0.1 sec, values for N greater than 10 (= 20, 25, 30) performed poorly around turns presumably because of the difficulty in predicting a path for longer than 1 sec (N x dt). Interestingly, N = 20 and dt = 0.05 sec provided very similar results but were computationally more expensive due to finer division of the predicted path. Smaller values of N (= 5, 7) failed to predict a path along the track in a straight line. Increasing dt (= 0.15 sec, 0.2 sec) for smaller values of N in order to get a reasonably long predicted path failed because the time steps were too long for the vehicle to react in time around corners. In test runs where it did react, it slowed down too much in order to avoid leaving the track.

## Polynomial Fitting and MPC Preprocessing

The waypoints were transformed from the map to vehicle coordinate system in order to make polynomial fitting easier at each time step ([main.cpp](https://github.com/nvphadnis/CarND-Term2-P5-ModelPredictiveControl/blob/master/src/main.cpp) lines 104-113). The vehicle position became the origin from where the polynomial as well as predicted path began while its orientation pointed towards the X axis.

## Model Predictive Control with Latency

The latency requirement was embedded into the state vector prior to passing it to the *mpc.Solve* function in order to calculate steering angle and throttle inputs to the simulator ([main.cpp](https://github.com/nvphadnis/CarND-Term2-P5-ModelPredictiveControl/blob/master/src/main.cpp) lines 130-141). The kinematic model used to predict state vectors in a future time step were used again to predict a state vector 100 ms later from when the vehicle was at its origin (x=0 and y=0) oriented along the X-axis (psi = 0) and moving with a constant velocity. This new state vector was passed to *mpc.Solve* as the “current” state vector. The output state vector would include the current steering angle and throttle inputs required for the future state vector that was fed into *mpc.Solve*, thus accounting for a latency delay in actuation.
