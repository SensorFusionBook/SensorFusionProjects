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

%--> Use the ekf state_correction_vector to correct all states 
for j=1:length(tracking_pose_indices)
    traget_idx = tracking_pose_indices(j)*10 - 9;
    
    pos_north_value(traget_idx) = pos_north_value(traget_idx) + state_correction_vector(31+j*7);
    pos_east_value(traget_idx) = pos_east_value(traget_idx) + state_correction_vector(32+j*7);
    pos_down_value(traget_idx) = pos_down_value(traget_idx) + state_correction_vector(33+j*7);
    pos_alt_value(traget_idx) = -pos_down_value(traget_idx);  
    
    pos_lat_value(traget_idx) = pos_lat_value(traget_idx) + state_correction_vector(32+j*7)/(RM_fun(pos_lat_value(traget_idx))+pos_alt_value(traget_idx));
    pos_lon_value(traget_idx) = pos_lon_value(traget_idx) + state_correction_vector(31+j*7)/((RN_fun(pos_lat_value(traget_idx))+pos_alt_value(traget_idx))*cos(pos_lat_value(traget_idx)));    
        
    delta_theta = 0.5.*[state_correction_vector(35+j*7),state_correction_vector(36+j*7), state_correction_vector(37+j*7)];
    delta_q = quatnormalize([1, delta_theta]);

    new_q = quatnormalize(quatmultiply([a_value(traget_idx), b_value(traget_idx), c_value(traget_idx), d_value(traget_idx)], delta_q));
    a_value(traget_idx) = new_q(1);
    b_value(traget_idx) = new_q(2);
    c_value(traget_idx) = new_q(3);
    d_value(traget_idx) = new_q(4);    
   
    [A, p, r] = quat2angle([a_value(traget_idx) b_value(traget_idx) c_value(traget_idx) d_value(traget_idx)],'ZYX');
    Euler_roll_value(traget_idx)      = r;
    Euler_pitch_value(traget_idx)     = p;
    Euler_heading_value(traget_idx)   = A;

       
    cam_pos_value(traget_idx, :) = [pos_north_value(traget_idx), pos_east_value(traget_idx), pos_down_value(traget_idx)]+... 
                                   (quat2rotm([a_value(traget_idx) b_value(traget_idx) c_value(traget_idx) d_value(traget_idx)])*cam_exts_value(index+1, 5:7)')';

    cam_att_value(traget_idx, :) = quatnormalize(quatmultiply([a_value(traget_idx) b_value(traget_idx) c_value(traget_idx) d_value(traget_idx)], cam_exts_value(index+1, 1:4)));

    [A, p, r] = quat2angle([cam_att_value(traget_idx, 1), cam_att_value(traget_idx, 2),cam_att_value(traget_idx, 3),cam_att_value(traget_idx, 4)],'ZYX');
    cam_roll_value(traget_idx)      = r;
    cam_pitch_value(traget_idx)     = p;
    cam_heading_value(traget_idx)   = A;    
    
end                 
