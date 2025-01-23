# Extended Kalman Filter

## Table of Contests

1. [File Descriptions](#file-descriptions)
2. [Usage Instructions](#usage-instructions)

## File decriptions
Prepared data in .mat extension (results1_data & results2_data) consists of only estimation results (340312x6). It is given to use this in section from "Results validation" till the end. Firstly load the test data contained in the pos_vel_data.mat file
- **'results1_data.mat'** & **'results2_data.mat'**:
  - Each row represents a signle estimation for position and velocity in X, Y, Z axis ([pX, pY, pZ, vX, vY, vZ])
  - Each column corresponds to a specified time value

## Usage Instructions
1. **Prerequisites**:
   - MATLAB or any tool capable of reading '.mat' files is required.
   - [Additional dependencies if needed].

2. **Loading the `.mat` File**:
   - To load the 'data.mat' file into MATLAB, use the following command:
   ```matlab
   load(`pos_vel_data.mat`);
   load(`results1_data.mat`);
