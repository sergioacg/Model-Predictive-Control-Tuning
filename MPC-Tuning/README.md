# Hybrid Tuning Algorithms for Model Predictive Controllers

This repository contains the hybrid tuning algorithms proposed in the following article:

> Giraldo, Sergio A. C., Príamo A. Melo, and Argimiro R. Secchi. 2022. "Tuning of Model Predictive Controllers Based on Hybrid optimization" Processes 10, no. 2:351. [https://doi.org/10.3390/pr10020351](https://doi.org/10.3390/pr10020351)

These codes have been adapted to work directly with the Matlab Model Predictive Control Toolbox. As a result, the dynamic outcomes may slightly differ from those presented in the paper. However, the same methodology is followed. This adaptation enables more agile code distribution and encourages improvements.


# MPC Controller Tuning Manual

This manual provides step-by-step instructions on how to use the MATLAB script for tuning an MPC controller using the MATLAB MPC Toolbox and the proposed hybrid optimization methodology. Please follow the steps below:

1. **Define the system model (linear or nonlinear) and sampling period (Ts)**: Specify the transfer function for your system (Psr) and input/output delay matrix (Lr). Set the sampling period (Ts) and the number of iterations (nit).

2. **Define the setpoint for the study**: Specify the setpoint values (Xsp) for the controlled variables in your system.

3. **Input a desired reference trajectory**: Provide a reference trajectory (Yref) that the optimization algorithm will attempt to match the output response to.

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


# MPCTuning Function

The main function of this library is `MPCTuning`, which finds the optimal parameters for an (N)MPC algorithm. The function has the following signature:

```matlab
function [mpcobj, scale, delta, lambda, Np, Nun] = MPCTuning(mpcobj_proj, Sp, linear, w, fi, Yref, mdv, nbp, nbc, nlmodel, init, optimset)
```

## Input Parameters

- `mpcobj_proj` (MPC object from the MPC toolbox): The input MPC object from the MPC toolbox that needs to be tuned.
- `Sp` (Internal setpoint): The internal setpoint used by the MPC controller.
- `linear` (Boolean): A boolean variable indicating if a linear system is used (`true`) or a non-linear system is used (`false`).
- `w` (Weight of Pareto optimization): The weight parameter used for Pareto optimization.
- `fi` (Iteration number): The number of iterations used for the optimization algorithm.
- `Yref` (Desired reference trajectory): The desired reference trajectory for the optimization algorithm.
- `mdv` (Stimulus of the MPC disturbance): Mainly used when working with an MPC control by bands where there is not a fixed setpoint in some   controlled variables (default: `mdv` = 1×0 empty double row vector).
- `nbp` (Number of bits for the prediction horizon): Default is 8 bits, which allows for a maximum prediction of 255.
- `nbc` (Number of bits for the control horizon): Default is 4 bits, which allows for a maximum prediction of 15.

## Optional Input Parameters (for non-linear systems)

- `nlmodel` (Non-linear model of the process): The non-linear model of the process to be integrated with an ODE solver.
- `init` (Initial condition for the `nlmodel`): A struct with the following fields:
  - `init.x0`: Initial condition of the model.
  - `init.xc`: Array with the output states of the model.
  - `init.u0`: Initial condition of the control action.
  - `init.integrator`: Model integration function (`@ode45`, `@ode15s`, etc.).
- `optimset` (Options for fgoalattain optimization): Configuration for the optimization process.

## Output Parameters

- `mpcobj` (Tuned MPC object): The MPC object from the MPC Toolbox with the tuning found and the internally scaled model and constraints. The user must scale the input and output signals of the MPC controller for correct calculations in the implementation.
- `scale` (Struct containing L and R matrices): Matrices for scaling the MPC signals.
- `delta` (Tracking setpoint weight): The optimal tracking setpoint weight found by the algorithm.
- `lambda` (Control increment weight): The optimal control increment weight found by the algorithm.
- `Np` (Prediction horizon): The optimal prediction horizon found by the algorithm.
- `Nun` (Control horizon): The optimal control horizon found by the algorithm.
- `Fob` (Final objective value): The final objective value of the optimization process.

For detailed information on how to use this function, please refer to the following article:

Giraldo, Sergio A. C., Príamo A. Melo, and Argimiro R. Secchi. 2022. "Tuning of Model Predictive Controllers Based on Hybrid Optimization" Processes 10, no. 2: 351. https://doi.org/10.3390/pr10020351


# System Scaling in MPC Controllers

When designing an MPC controller, whether for linear or nonlinear cases, it is essential to properly scale the system. This helps to avoid ill-conditioning issues in the matrices, improves numerical stability, and leads to better controller performance.

## Linear Case

For linear MPC implementation, the system model can be scaled by minimizing the conditioning number of the matrix G(z), where G(z) represents the linear model of the system, y(z) = G(z)u(z):

$$
\underset{\mathbf{L,R}}{\rm min}\ \beta[\mathbf{L}\mathbf{G}(z)\mathbf{R}],
$$


Here, L and R are diagonal matrices, and β[G] is the conditioning number of the matrix G. The scaled model, y_s(z) = Ly(z) and u_s(z) = R^{-1}u(z), obtained from the equation above, is used for simulation and controller design.

## Nonlinear Case (NMPC)

For the nonlinear case using NMPC, we can employ the ScaleFactor property of the nlmpc object. Here's an explanation of how the ranges Urange, Yrange, and Xrange are determined:

1. Urange: This range is determined by calculating the difference between the maximum and minimum values of the manipulated variables (umax - umin).

2. Yrange: This range is calculated by taking the difference between the maximum and minimum values of the controlled variables (xmax(xc) - xmin(xc)).

3. Xrange: This range is calculated by taking the difference between the maximum and minimum values of the state variables (xmax - xmin).

The ScaleFactor property is then set for each manipulated variable, output variable, and state variable in the nlmpc object.

Here is an example of setting the ScaleFactors for the manipulated variables, output variables, and state variables:

\`\`\`matlab
% NMPC Scales
% set input and output range
Urange = umax-umin;
Yrange = xmax(xc)-xmin(xc);
Xrange = xmax - xmin;
% scale manipulated variables
for i = 1:nu
    nlobj_proj.ManipulatedVariables(i).ScaleFactor = Urange(i);
end
% scale outputs
for i = 1:ny
    nlobj_proj.OV(i).ScaleFactor = Yrange(i);
end
% scale states
for i = 1:nx
    nlobj_proj.States(i).ScaleFactor = Xrange(i);
end
\`\`\`

By properly scaling the system, the controller can achieve better performance and numerical stability.


