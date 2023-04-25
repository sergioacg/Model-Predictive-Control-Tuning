# Hybrid Tuning Algorithms for Model Predictive Controllers

This repository contains the hybrid tuning algorithms proposed in the following article:

> Giraldo, Sergio A. C., Príamo A. Melo, and Argimiro R. Secchi. 2022. "Tuning of Model Predictive Controllers Based on Hybrid optimization" Processes 10, no. 2:351. [https://doi.org/10.3390/pr10020351](https://doi.org/10.3390/pr10020351)

These codes have been adapted to work directly with the Matlab Model Predictive Control Toolbox. As a result, the dynamic outcomes may slightly differ from those presented in the paper. However, the same methodology is followed. This adaptation enables more agile code distribution and encourages improvements.


# MPC Controller Tuning Manual

This manual provides step-by-step instructions on how to use the MATLAB script for tuning an MPC controller using the MATLAB MPC Toolbox and the proposed hybrid optimization methodology. Please follow the steps below:

1. **Define the system model (linear or nonlinear) and sampling period (Ts)**: Specify the transfer function for your system (Psr) and input/output delay matrix (Lr). Set the sampling period (Ts) and the number of iterations (nit).

2. **Define the setpoint for the study**: Specify the setpoint values (Xsp) for the controlled variables in your system.

3. **Input a desired reference trajectory**: Provide a reference trajectory (Pref) that the optimization algorithm will attempt to match the output response to.

4. **Create the MPC controller object**: Set up the MPC controller object using the discrete-time model (sysd) and the specified sample time (Ts).

5. **Specify nominal values for inputs and outputs**: Define nominal values for the plant inputs and outputs in the MPC object (mpc_toolbox.Model.Nominal).

6. **Specify constraints**: Define constraints for the MPC controller inputs and outputs (InUmx, InUmn, Umx, Umn, Ymx, Ymn).

7. **Set constraints for MV, MV Rate, and OV**: Set constraints on the control inputs, their increments, and the plant outputs in the MPC object.

8. **Specify simulation options**: Set simulation options for the MPC controller using the 'mpcsimopt' function.

9. **Call the MPCTuning algorithm**: Call the MPCTuning function with the required input arguments (mpcobj, Sp, linear, w, fi, Pref). You may also provide optional input arguments for non-linear systems (nbp, nbc, nlmodel, init, optimset). The MPCTuning function finds the optimal values for the MPC algorithm parameters (delta, lambda, Np, Nun).

The provided script includes code for defining the system model, setting up the MPC controller object, specifying constraints, and calling the MPCTuning algorithm. Follow these instructions along with the script to tune your MPC controller using the proposed hybrid optimization methodology.


### Nonlinear Case: Additional Instructions

In the nonlinear case, two additional scripts must be generated depending on the model. One script should contain the model (e.g., differential equations) and another script should contain the states.

For the nonlinear case, follow these additional instructions:

1. **Create two additional scripts**: One script should include the nonlinear model (e.g., in the form of differential equations), and the other script should contain the states of the system.

2. **Define the model and state functions**: In the first script, define a function that represents the nonlinear model (e.g., `vandevusse_model`). In the second script, define a function that represents the states (e.g., `nmpc_vandevusse_state`).

3. **Define the initial conditions and constraints**: Define the initial conditions for the inputs and the system states, as well as constraints on the control inputs, their increments, and the plant outputs.

4. **Create a nonlinear MPC (NMPC) object**: Set up an NMPC object using the MATLAB NMPC Toolbox, and define the internal model, output function, and the constraints for the NMPC object.

5. **Specify the reference trajectory**: Define the desired response of the controller (e.g., as a first-order dynamics with unit gain) and convert it to the discrete-time domain using `c2d` function.

6. **Call the MPCTuning algorithm**: Call the MPCTuning function with the required input arguments (nlobj, r, lineal, w, nit, Prefz, 5, 4, model, init). The MPCTuning function finds the optimal values for the MPC algorithm parameters (delta, lambda, N, Nu).

The provided script includes code for defining the nonlinear model, setting up the NMPC object, specifying constraints, and calling the MPCTuning algorithm. Follow these instructions along with the script to tune your NMPC controller using the proposed hybrid optimization methodology.
