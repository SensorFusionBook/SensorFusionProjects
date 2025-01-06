%{
Visual-Inertial Fusion Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

The algorithm is described in the following papers:

[1] S. Sheikhpour and M. M. Atia, "An Enhanced Visual-Inertial Navigation 
System Based on Multi-State Constraint Kalman Filter," 
IEEE 63rd Proceedings of Midwest Symposium on Circuits and Systems (MWSCAS)
, pp. 361-364,MA, USA, 2020, (https://ieeexplore.ieee.org/document/9184501)

[2] S. Sheikhpour and M. M. Atia "A Real-Time CPU-GPU Embedded
Implementation of a Tightly-Coupled Visual-Inertial Navigation System",
IEEE Access, Vol. 10, pp: 86384 - 86394, 17 August 2022
(https://ieeexplore.ieee.org/abstract/document/9858052)

%}

%--> Update the vehicle shape based on the updated position and orientation
C_LB_value_plus_90 = C_LB_Euler_fun(Euler_roll_value(index+1),-Euler_pitch_value(index+1),pi/2 - Euler_heading_value(index+1));
updated_vert = C_LB_value_plus_90*initial_vert';
updated_vert = updated_vert';
for p = 1:length(updated_vert)
    updated_vert(p,:) = updated_vert(p,:) + [pos_east_value(index), pos_north_value(index), pos_alt_value(index)];
end
[ CubeXData , CubeYData , CubeZData ] = get_cube_axis_data(updated_vert);
CubePoints = [updated_vert(:,1),updated_vert(:,2),updated_vert(:,3)];
