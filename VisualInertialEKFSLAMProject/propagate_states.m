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

accel_x_meas(index) = dataset.accel(index,1);
accel_y_meas(index) = dataset.accel(index,2);
accel_z_meas(index) = dataset.accel(index,3);

gyro_x_meas(index) = dataset.gyro(index,1);
gyro_y_meas(index) = dataset.gyro(index,2);
gyro_z_meas(index) = dataset.gyro(index,3);

%--> attitude equations
a_dot_value = a_dot_fun(IMU_q_BI_value(index,1),IMU_q_BI_value(index,2),IMU_q_BI_value(index,3),IMU_q_BI_value(index,4),a_value(index),b_value(index),c_value(index),d_value(index),gyro_x_meas(index),gyro_y_meas(index),gyro_z_meas(index),gyro_x_std_value,gyro_y_std_value,gyro_z_std_value,gyro_x_bias_value(index),gyro_y_bias_value(index),gyro_z_bias_value(index),gyro_x_scale_value(index),gyro_y_scale_value(index),gyro_z_scale_value(index),pos_alt_value(index),pos_lat_value(index),ve_value(index),vn_value(index),wg_noise_value);
b_dot_value = b_dot_fun(IMU_q_BI_value(index,1),IMU_q_BI_value(index,2),IMU_q_BI_value(index,3),IMU_q_BI_value(index,4),a_value(index),b_value(index),c_value(index),d_value(index),gyro_x_meas(index),gyro_y_meas(index),gyro_z_meas(index),gyro_x_std_value,gyro_y_std_value,gyro_z_std_value,gyro_x_bias_value(index),gyro_y_bias_value(index),gyro_z_bias_value(index),gyro_x_scale_value(index),gyro_y_scale_value(index),gyro_z_scale_value(index),pos_alt_value(index),pos_lat_value(index),ve_value(index),vn_value(index),wg_noise_value);
c_dot_value = c_dot_fun(IMU_q_BI_value(index,1),IMU_q_BI_value(index,2),IMU_q_BI_value(index,3),IMU_q_BI_value(index,4),a_value(index),b_value(index),c_value(index),d_value(index),gyro_x_meas(index),gyro_y_meas(index),gyro_z_meas(index),gyro_x_std_value,gyro_y_std_value,gyro_z_std_value,gyro_x_bias_value(index),gyro_y_bias_value(index),gyro_z_bias_value(index),gyro_x_scale_value(index),gyro_y_scale_value(index),gyro_z_scale_value(index),pos_alt_value(index),pos_lat_value(index),ve_value(index),vn_value(index),wg_noise_value);
d_dot_value = d_dot_fun(IMU_q_BI_value(index,1),IMU_q_BI_value(index,2),IMU_q_BI_value(index,3),IMU_q_BI_value(index,4),a_value(index),b_value(index),c_value(index),d_value(index),gyro_x_meas(index),gyro_y_meas(index),gyro_z_meas(index),gyro_x_std_value,gyro_y_std_value,gyro_z_std_value,gyro_x_bias_value(index),gyro_y_bias_value(index),gyro_z_bias_value(index),gyro_x_scale_value(index),gyro_y_scale_value(index),gyro_z_scale_value(index),pos_alt_value(index),pos_lat_value(index),ve_value(index),vn_value(index),wg_noise_value);

if (~isreal([a_dot_value b_dot_value c_dot_value d_dot_value]))
    disp('imaginary quaternion rate quat_dot');return;
end

%--> Advance quaternion
% Simple integration
a_value(index+1) = a_value(index) + a_dot_value*IMU_sampling_period_sec;
b_value(index+1) = b_value(index) + b_dot_value*IMU_sampling_period_sec;
c_value(index+1) = c_value(index) + c_dot_value*IMU_sampling_period_sec;
d_value(index+1) = d_value(index) + d_dot_value*IMU_sampling_period_sec;

% normalization
normalized_quat = quatnormalize([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)]);
a_value(index+1) = normalized_quat(1);
b_value(index+1) = normalized_quat(2);
c_value(index+1) = normalized_quat(3);
d_value(index+1) = normalized_quat(4);

%--> Transforming to Euler for 3D graphics
[A, p, r] = quat2angle([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)],'ZYX');
Euler_roll_value(index+1)      = r;
Euler_pitch_value(index+1)     = p;
Euler_heading_value(index+1)   = A;

if (~isreal([Euler_roll_value(index+1) Euler_pitch_value(index+1) Euler_heading_value(index+1)]))
    disp('imaginary Euler angles');return;
end        

%--> Advance velocity
vel_dot_value = vel_dot_fun(IMU_q_BI_value(index,1), IMU_q_BI_value(index,2),IMU_q_BI_value(index,3),IMU_q_BI_value(index,4),a_value(index),accel_x_meas(index),accel_y_meas(index),accel_z_meas(index),accel_x_std_value,accel_y_std_value,accel_z_std_value,accel_x_bias_value(index),accel_y_bias_value(index),accel_z_bias_value(index),accel_x_scale_value(index),accel_y_scale_value(index),accel_z_scale_value(index),b_value(index),c_value(index),d_value(index),pos_alt_value(index),pos_lat_value(index),vd_value(index),ve_value(index),vn_value(index),wg_noise_value);

vn_value(index+1) = vn_value(index) + vel_dot_value(1)*IMU_sampling_period_sec;
ve_value(index+1) = ve_value(index) + vel_dot_value(2)*IMU_sampling_period_sec;
vd_value(index+1) = vd_value(index) + vel_dot_value(3)*IMU_sampling_period_sec;
vu_value(index+1) = -vd_value(index+1);

%--> Advance position (Trapizoidal integration)
pos_east_value(index+1) = pos_east_value(index) + (ve_value(index)+ve_value(index+1))*IMU_sampling_period_sec*0.5;
pos_north_value(index+1) = pos_north_value(index) + (vn_value(index)+vn_value(index+1))*IMU_sampling_period_sec*0.5;

pos_down_value(index+1)  = pos_down_value(index) + (vd_value(index)+vd_value(index+1))*IMU_sampling_period_sec*0.5;
pos_alt_value(index+1) = -pos_down_value(index+1);

pos_lat_value(index+1) = pos_lat_value(index) + ((vn_value(index)/(RM_fun(pos_lat_value(index))+pos_alt_value(index)))+...
                                                 (vn_value(index+1)/(RM_fun(pos_lat_value(index))+pos_alt_value(index+1))))*IMU_sampling_period_sec*0.5;

pos_lon_value(index+1) = pos_lon_value(index) + ((ve_value(index)/((RN_fun(pos_lat_value(index))+pos_alt_value(index))*cos(pos_lat_value(index))))+...
                                                 (ve_value(index+1)/((RN_fun(pos_lat_value(index+1))+pos_alt_value(index+1))*cos(pos_lat_value(index+1)))))*IMU_sampling_period_sec*0.5;


%--> Advance IMU model states
gyro_x_bias_value(index+1) = gyro_x_bias_value(index);
gyro_y_bias_value(index+1) = gyro_y_bias_value(index);
gyro_z_bias_value(index+1) = gyro_z_bias_value(index);

gyro_x_scale_value(index+1) = gyro_x_scale_value(index);
gyro_y_scale_value(index+1) = gyro_y_scale_value(index);
gyro_z_scale_value(index+1) = gyro_z_scale_value(index);

accel_x_bias_value(index+1) = accel_x_bias_value(index);
accel_y_bias_value(index+1) = accel_y_bias_value(index);
accel_z_bias_value(index+1) = accel_z_bias_value(index);

accel_x_scale_value(index+1) = accel_x_scale_value(index);
accel_y_scale_value(index+1) = accel_y_scale_value(index);
accel_z_scale_value(index+1) = accel_z_scale_value(index);

IMU_q_BI_value(index+1, :) = IMU_q_BI_value(index, :);

%--> Advance camera model states
cam_ints_value(index+1,:) =  cam_ints_value(index,:);
cam_exts_value(index+1,:) =  cam_exts_value(index,:);

%--> Tranform the IMU frame pose to the Camera frame pose
cam_pos_value(index+1, :) = [pos_north_value(index+1), pos_east_value(index+1), pos_down_value(index+1)] +... 
                            (quat2rotm([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)])*cam_exts_value(index+1, 5:7)')';
                        
cam_att_value(index+1, :) = quatnormalize(quatmultiply([a_value(index+1) b_value(index+1) c_value(index+1) d_value(index+1)], cam_exts_value(index+1, 1:4)));

[A, p, r] = quat2angle([cam_att_value(index+1, 1), cam_att_value(index+1, 2),cam_att_value(index+1, 3),cam_att_value(index+1, 4)],'ZYX');
cam_roll_value(index+1)      = r;
cam_pitch_value(index+1)     = p;
cam_heading_value(index+1)   = A;
