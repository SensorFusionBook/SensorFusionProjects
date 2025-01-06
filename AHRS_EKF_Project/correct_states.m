%{
Attitude and Heading Reference System (AHRS) Project using EKF
Sensors include Magnetometer, Accelerometer, and Gyroscope
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

%--> Apply updates
b_value(index+1) = b_value(index+1) + state_correction_vector(7);
c_value(index+1) = c_value(index+1) + state_correction_vector(8);
d_value(index+1) = d_value(index+1) + state_correction_vector(9);
a_value(index+1) = a_value(index+1) - ([b_value(index) c_value(index) d_value(index)]*state_correction_vector(7:9))/a_value(index);

gyro_bias_x_value(index+1) = gyro_bias_x_value(index+1) + state_correction_vector(10);
gyro_bias_y_value(index+1) = gyro_bias_y_value(index+1) + state_correction_vector(11);
gyro_bias_z_value(index+1) = gyro_bias_z_value(index+1) + state_correction_vector(12);
gyro_sf_x_value(index+1) = gyro_sf_x_value(index+1) + state_correction_vector(13);
gyro_sf_y_value(index+1) = gyro_sf_y_value(index+1) + state_correction_vector(14);
gyro_sf_z_value(index+1) = gyro_sf_z_value(index+1) + state_correction_vector(15);

acc_bias_x_value(index+1) = acc_bias_x_value(index+1) + state_correction_vector(16);
acc_bias_y_value(index+1) = acc_bias_y_value(index+1) + state_correction_vector(17);
acc_bias_z_value(index+1) = acc_bias_z_value(index+1) + state_correction_vector(18);
acc_sf_x_value(index+1) = acc_sf_x_value(index+1) + state_correction_vector(19);
acc_sf_y_value(index+1) = acc_sf_y_value(index+1) + state_correction_vector(20);
acc_sf_z_value(index+1) = acc_sf_z_value(index+1) + state_correction_vector(21);

%--> Normalize quaternion and update Euler angles
q = [a_value(index+1), b_value(index+1), c_value(index+1), d_value(index+1)];
a_value(index+1) = q(1);
b_value(index+1) = q(2);
c_value(index+1) = q(3);
d_value(index+1) = q(4);

[Heading, pitch, roll] = quat2angle(q,'ZYX');

Euler_roll_value(index + 1) = roll;
Euler_pitch_value(index + 1) = pitch;
Euler_heading_value(index + 1) = Heading;
