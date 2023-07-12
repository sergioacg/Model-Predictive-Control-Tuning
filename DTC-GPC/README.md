# MIMO DTC-GPC for Wood and Berry Column 

**Author:** Sergio Andres Castaño Giraldo

**Website:** [Control Automatic Education](http://controlautomaticoeducacion.com/)

**Institution:** Federal University of Rio de Janeiro

**Year:** 2018

## Description

This MATLAB script performs a MIMO (Multiple Input Multiple Output) DTC-GPC (Dynamic Matrix Control - Generalized Predictive Control) for a Wood and Berry column using the Diophantine method. The example implements a MIMO 2x2 system, with variables y_1, y_2 representing the process outputs, and u_1, u_2 representing the control actions. The script utilizes the optimal predictor and the minimal delays of the fast model for the calculation of past controls. 

## How to Use

Ensure you have MATLAB installed on your machine.

1. Download the .m file from this repository.
2. Open MATLAB and navigate to the directory where you saved the .m file.
3. Run the .m file in MATLAB.

## Code Overview

The script performs the following steps:

- Defines the process of the column and introduces uncertainties for simulation.
- Establishes a system with delay and uncertainty.
- Conditions the model and nominal model of the process.
- Discretizes the models and defines a fast model.
- Sets GPC tuning parameters and weights matrices.
- Solves the Diophantine equation and creates the matrix of the forced response.
- Determines the vector with past controls.
- Defines a Fr and S Filter.
- Starts a control loop, calculating the optimal predictor, the free response, and control increment within each iteration.
- The process is visualized using MATLAB's plot functions to show the process output and control action over time.

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.
