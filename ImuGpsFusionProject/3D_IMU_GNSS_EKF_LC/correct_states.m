%{
Imu Gps Fusion Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

lat_value(index+1) = (lat_value(index+1)*D2R + state_correction_vector(2)/(Rm_value(index+1)+alt_value(index+1)))*R2D;
lon_value(index+1) = (lon_value(index+1)*D2R + state_correction_vector(1)/((Rm_value(index+1)+alt_value(index+1))*cos(lat_value(index+1)*D2R)))*R2D;
% C_EN_value = C_EN_fun(lat_value(index+1)*D2R , lon_value(index+1)*D2R);
% pos_quat_vector = convert_dcm_to_quat (C_EN_value);

% Get C_EN dcm matrix from lat and lon
lon_in = lon_value(index+1)*D2R;
lat_in = lat_value(index+1)*D2R;
C_EN_value(1,1) = cos(lon_in);C_EN_value(1,2) = - sin(lon_in)*sin(lat_in);C_EN_value(1,3) = sin(lon_in)*cos(lat_in);
C_EN_value(2,1) = 0; C_EN_value(2,2) = cos(lat_in); C_EN_value(2,3) = sin(lat_in);
C_EN_value(3,1) = -sin(lon_in); C_EN_value(3,2) = -cos(lon_in)*sin(lat_in); C_EN_value(3,3) = cos(lon_in)*cos(lat_in);
% convert position dcm to position quaternion
pos_quat_vector = dcm2quat( C_EN_value' );

a_pos_value(index+1) = pos_quat_vector(1);
b_pos_value(index+1) = pos_quat_vector(2);
c_pos_value(index+1) = pos_quat_vector(3);
d_pos_value(index+1) = pos_quat_vector(4);
normalization_factor = sqrt(a_pos_value(index+1)^2 + b_pos_value(index+1)^2 + c_pos_value(index+1)^2 + d_pos_value(index+1)^2);
a_pos_value(index+1) = a_pos_value(index+1)/normalization_factor;
b_pos_value(index+1) = b_pos_value(index+1)/normalization_factor;
c_pos_value(index+1) = c_pos_value(index+1)/normalization_factor;
d_pos_value(index+1) = d_pos_value(index+1)/normalization_factor;
alt_value(index+1) = alt_value(index+1) + state_correction_vector(3);
ve_value(index+1) = ve_value(index+1) + state_correction_vector(4);
vn_value(index+1) = vn_value(index+1) + state_correction_vector(5);
vu_value(index+1) = vu_value(index+1) + state_correction_vector(6);
b_value(index+1) = b_value(index+1) + state_correction_vector(7);
c_value(index+1) = c_value(index+1) + state_correction_vector(8);
d_value(index+1) = d_value(index+1) + state_correction_vector(9);
a_value(index+1) = a_value(index+1) - ([b_value(index) c_value(index) d_value(index)]*state_correction_vector(7:9))/a_value(index);
gyro_bias_x_value(index+1) = gyro_bias_x_value(index+1) + state_correction_vector(10);
gyro_bias_y_value(index+1) = gyro_bias_y_value(index+1) + state_correction_vector(11);
gyro_bias_z_value(index+1) = gyro_bias_z_value(index+1) + state_correction_vector(12);
acc_bias_x_value(index+1) = acc_bias_x_value(index+1) + state_correction_vector(13);
acc_bias_y_value(index+1) = acc_bias_y_value(index+1) + state_correction_vector(14);
acc_bias_z_value(index+1) = acc_bias_z_value(index+1) + state_correction_vector(15);
gyro_sf_x_value(index+1) = gyro_sf_x_value(index+1) + state_correction_vector(16);
gyro_sf_y_value(index+1) = gyro_sf_y_value(index+1) + state_correction_vector(17);
gyro_sf_z_value(index+1) = gyro_sf_z_value(index+1) + state_correction_vector(18);
acc_sf_x_value(index+1) = acc_sf_x_value(index+1) + state_correction_vector(19);
acc_sf_y_value(index+1) = acc_sf_y_value(index+1) + state_correction_vector(20);
acc_sf_z_value(index+1) = acc_sf_z_value(index+1) + state_correction_vector(21);

% Normalize the quaternions
normalization_factor = sqrt(a_value(index+1)^2 + b_value(index+1)^2 + c_value(index+1)^2 + d_value(index+1)^2);
a_value(index+1) = a_value(index+1)/normalization_factor;
b_value(index+1) = b_value(index+1)/normalization_factor;
c_value(index+1) = c_value(index+1)/normalization_factor;
d_value(index+1) = d_value(index+1)/normalization_factor;
normalization_factor = sqrt(a_pos_value(index+1)^2 + b_pos_value(index+1)^2 + c_pos_value(index+1)^2 + d_pos_value(index+1)^2);
a_pos_value(index+1) = a_pos_value(index+1)/normalization_factor;
b_pos_value(index+1) = b_pos_value(index+1)/normalization_factor;
c_pos_value(index+1) = c_pos_value(index+1)/normalization_factor;
d_pos_value(index+1) = d_pos_value(index+1)/normalization_factor;

% Calculate DCM from Quaternion
C_LB_from_quat_value = quat2dcm( [a_value(index+1); b_value(index+1); c_value(index+1); d_value(index+1)]' );
C_LB_from_quat_value = C_LB_from_quat_value';

% Calculate Euler angles from quaternion
Euler_pitch_value(index+1)     = atan2(-C_LB_from_quat_value(3,1),(sqrt(C_LB_from_quat_value(3,2)^2 + C_LB_from_quat_value(3,3)^2)));
Euler_roll_value(index+1)      = atan2(C_LB_from_quat_value(3,2),C_LB_from_quat_value(3,3));
Euler_heading_value(index+1)   = atan2(C_LB_from_quat_value(2,1),C_LB_from_quat_value(1,1));
Rn_value(index+1)  = earth_a/sqrt(1-earth_e2*sin(lat_value(index+1)*D2R)*sin(lat_value(index+1)*D2R));
Rm_value(index+1)  = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat_value(index+1)*D2R)*sin(lat_value(index+1)*D2R))^(1.5));
EP_value(index+1) = (lon_value(index+1)-lon_value(1))*D2R*(Rn_value(index+1)+alt_value(index+1))*cos(lat_value(index+1)*D2R);
NP_value(index+1) = (lat_value(index+1)-lat_value(1))*D2R*(Rm_value(index+1)+alt_value(index+1));