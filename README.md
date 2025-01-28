# Extended Kalman Filter

## Table of Contests

1. [General Description](#general-description)
2. [Usage Instructions](#usage-instructions)
3. [File Descriptions](#file-descriptions)
4. [Results](#results)

## General Description
This section is dedicated to the execution of Extended Kalman Filter (EKF) in ECEF sphere. This instruction consists of file decsriptions, usage instructions and results.

![Satelite trajectory ECEF](/screenshots/trajectory.jpg)

As a refference data the FCC measurements were used and as a correction data GPS measurements were used.

![DataPosition](/screenshots/measurement_position.jpg)
![DataVelocity](/screenshots/measurement_velocity.jpg)

Specified describtion how to use this scripts, equations and results are presented in [EKF.pdf](/docs/EKF.pdf)

## Usage Instructions
1. **Prerequisites**:
   - MATLAB 2024b+ or any tool capable of reading (`.m`, `.mat`) files is required

2. **Execution**:
   - Open `EKF.m` to estimate position and velocity of a satelite in [X,Y,Z] axis 
   - `EKF.m` uses 7 auxiliary functions that are added to [/src](/src) folder
   - Two tests were executed with changed `STPu` parameter (steps per time unit). This is in [/tests](/tests) folder, to be able to validate results, load data after main loop section
   - To load the `data.mat` file into MATLAB, use the following command:
   ```matlab
   load('data.mat');

## File Decriptions
### Scripts:
- [EKF.m](/src/EKF.m) performs parameters estimation using EKF algorithm
- [correction.m](/src/correction.m) performs a correction step based on the last GPS measurement
- [get_jacob.m](/src/get_jacob.m) calculates the Jacobi matrix for the state vector
- [matrix_to_vector.m](/src/matrix_to_vector.m) rewrites a matrix into a vector 
- [rhs.m](/src/rhs.m) calculates the derivatives of the state equations
- [rhs_ekf.m](/src/rhs_ekf.m) returns the derivative of the combined state vector
- [rk4.m](/src/rk4.m) performs RK4 integration
- [vector_to_matrix.m](/src/vector_to_matrix.m) rewrites a vector into a matrix

ATTENTION! There is no need to open positions from 2 to 8. They are appropriately used in [EKF.m](/src/EKF.m) as auxiliary functions.

### Data:
- [pos_vel_data.mat](/src/pos_vel_data.mat) data with GPS (correction) measurements and with FCC (refference) measurements
- [results1_data.mat](/tests/results1_data.mat) data of executed EKF prediction with STPu = 10
- [results2_data.mat](/tests/results2_data.mat) data of executed EKF prediction with STPu = 100

## Results
Results are stored in [EKF.pdf](/docs/EKF.pdf) file.
