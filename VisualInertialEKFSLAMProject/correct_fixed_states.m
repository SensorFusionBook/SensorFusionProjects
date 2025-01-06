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
pos_north_value(index+1) = pos_north_value(index+1) + state_correction_vector(1);
pos_east_value(index+1) = pos_east_value(index+1) + state_correction_vector(2);
pos_down_value(index+1) = pos_down_value(index+1) + state_correction_vector(3);
pos_alt_value(index+1) = -pos_down_value(index+1);

pos_lat_value(index+1) = pos_lat_value(index+1) + state_correction_vector(2)/(RM_fun(pos_lat_value(index+1))+pos_alt_value(index+1));
pos_lon_value(index+1) = pos_lon_value(index+1) + state_correction_vector(1)/((RN_fun(pos_lat_value(index+1))+pos_alt_value(index+1))*cos(pos_lat_value(index+1)));

vn_value(index+1) = vn_value(index+1) + state_correction_vector(4);
ve_value(index+1) = ve_value(index+1) + state_correction_vector(5);
vd_value(index+1) = vd_value(index+1) + state_correction_vector(6);

delta_theta = 0.5.*[state_correction_vector(8), state_correction_vector(9), state_correction_vector(10)];
delta_q = quatnormalize([1, delta_theta]);

new_q = quatnormalize(quatmultiply([a_value(index+1), b_value(index+1), c_value(index+1), d_value(index+1)], delta_q));
a_value(index+1) = new_q(1);
b_value(index+1) = new_q(2);
c_value(index+1) = new_q(3);
d_value(index+1) = new_q(4);

[A, p, r] = quat2angle([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)],'ZYX');
Euler_roll_value(index+1)      = r;
Euler_pitch_value(index+1)     = p;
Euler_heading_value(index+1)   = A;

gyro_x_bias_value(index+1) = gyro_x_bias_value(index+1) + state_correction_vector(11);
gyro_y_bias_value(index+1) = gyro_y_bias_value(index+1) + state_correction_vector(12);
gyro_z_bias_value(index+1) = gyro_z_bias_value(index+1) + state_correction_vector(13);

accel_x_bias_value(index+1) = accel_x_bias_value(index+1) + state_correction_vector(14);
accel_y_bias_value(index+1) = accel_y_bias_value(index+1) + state_correction_vector(15);
accel_z_bias_value(index+1) = accel_z_bias_value(index+1) + state_correction_vector(16);

gyro_x_scale_value(index+1) = gyro_x_scale_value(index+1) + state_correction_vector(17);
gyro_y_scale_value(index+1) = gyro_y_scale_value(index+1) + state_correction_vector(18);
gyro_z_scale_value(index+1) = gyro_z_scale_value(index+1) + state_correction_vector(19);

accel_x_scale_value(index+1) = accel_x_scale_value(index+1) + state_correction_vector(20);
accel_y_scale_value(index+1) = accel_y_scale_value(index+1) + state_correction_vector(21);
accel_z_scale_value(index+1) = accel_z_scale_value(index+1) + state_correction_vector(22);

delta_theta = 0.5.*[state_correction_vector(24), state_correction_vector(25), state_correction_vector(26)];
delta_q = quatnormalize([1, delta_theta]);

new_q = quatnormalize(quatmultiply(IMU_q_BI_value(index+1,:), delta_q));
IMU_q_BI_value(index+1,:) = new_q;

cam_ints_value(index+1,:) =  cam_ints_value(index+1,:) + state_correction_vector(27:30)';

delta_theta = 0.5.*[state_correction_vector(32), state_correction_vector(33), state_correction_vector(34)];
delta_q = quatnormalize([1, delta_theta]);

new_q = quatnormalize(quatmultiply([cam_exts_value(index+1,1), cam_exts_value(index+1,2), cam_exts_value(index+1,3), cam_exts_value(index+1,4)], delta_q));
cam_exts_value(index+1,1) = new_q(1);
cam_exts_value(index+1,2) = new_q(2);
cam_exts_value(index+1,3) = new_q(3);
cam_exts_value(index+1,4) = new_q(4);

cam_exts_value(index+1,5:7) =  cam_exts_value(index+1, 5:7) + state_correction_vector(35:37)';

cam_pos_value(index+1, :) = [pos_north_value(index+1), pos_east_value(index+1), pos_down_value(index+1)]+... 
                            (quat2rotm([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)])*cam_exts_value(index+1, 5:7)')';
                                                 
cam_att_value(index+1, :) = quatnormalize(quatmultiply([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)], cam_exts_value(index+1, 1:4)));

[A, p, r] = quat2angle([cam_att_value(index+1, 1), cam_att_value(index+1, 2),cam_att_value(index+1, 3),cam_att_value(index+1, 4)],'ZYX');
cam_roll_value(index+1)      = r;
cam_pitch_value(index+1)     = p;
cam_heading_value(index+1)   = A;
