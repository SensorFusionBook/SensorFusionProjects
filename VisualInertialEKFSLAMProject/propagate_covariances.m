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

%--> Kalman Filter model matrices calculation 
F_fixed_value = F_fixed_fun(IMU_q_BI_value(index,1),IMU_q_BI_value(index,2),IMU_q_BI_value(index,3),IMU_q_BI_value(index,4),...
                        a_value(index),...
                        accel_x_meas(index),accel_y_meas(index),accel_z_meas(index),...
                        accel_x_std_value,accel_y_std_value,accel_z_std_value,accel_x_bias_value(index),accel_y_bias_value(index),accel_z_bias_value(index),...
                        accel_x_scale_value(index),accel_y_scale_value(index),accel_z_scale_value(index),...
                        b_value(index),c_value(index),d_value(index),...
                        gyro_x_meas(index),gyro_y_meas(index),gyro_z_meas(index),...
                        gyro_x_std_value,gyro_y_std_value,gyro_z_std_value,gyro_x_bias_value(index),gyro_y_bias_value(index),gyro_z_bias_value(index),...
                        gyro_x_scale_value(index),gyro_y_scale_value(index),gyro_z_scale_value(index),...    
                        pos_alt_value(index),pos_lat_value(index),vd_value(index), ve_value(index),vn_value(index),wg_noise_value);

if (~isreal(F_fixed_value))
    disp('imaginary transition matrix');return;
end

Phi_fixed = eye(size(F_fixed_value)) + F_fixed_value*IMU_sampling_period_sec;

G_fixed_value = G_fixed_fun(a_value(index), accel_x_std_value,accel_y_std_value,accel_z_std_value,...
                        accel_x_bias_std_value,accel_y_bias_std_value,accel_z_bias_std_value,...
                        accel_x_scale_std_value,accel_y_scale_std_value,accel_z_scale_std_value,...
                        b_value(index),c_value(index),d_value(index),...
                        gyro_x_std_value,gyro_y_std_value,gyro_z_std_value,...
                        gyro_x_bias_std_value,gyro_y_bias_std_value,gyro_z_bias_std_value,...
                        gyro_x_scale_std_value,gyro_y_scale_std_value,gyro_z_scale_std_value);

                    

if (~isreal(G_fixed_value))
    disp('imaginary noise shaping matrix');return;
end

%--> Discrete system covariance matrix from Aided Navigaotpn book
Q_d_fixed = IMU_sampling_period_sec*(G_fixed_value*G_fixed_value');

%--> EKF prediction
P_fixed = P(1:fixed_state_list_length,1:fixed_state_list_length);

P_fixed = Phi_fixed*P_fixed*Phi_fixed' + Q_d_fixed;
% P_fixed = Phi_fixed*P_fixed*Phi_fixed';

P(1:fixed_state_list_length,1:fixed_state_list_length) = P_fixed; 

P(1:fixed_state_list_length, fixed_state_list_length+1:end) = Phi_fixed*P(1:fixed_state_list_length, fixed_state_list_length+1:end);
P(fixed_state_list_length+1:end, 1:fixed_state_list_length) = P(1:fixed_state_list_length, fixed_state_list_length+1:end)';

%--> Fixing Positive-Definiteness of P
P = (P+P')/2;

if (isreal(P))
    [V, D] = eig(P); % Calculate the eigende composition of P matrix
    d= diag(D);
    d(d <= 1e-7) = 1e-7; % Set any eigenvalues that are lower than threshold "TH" to a fixed non-zero "small" value 
    P = V*diag(d)*V'; % Recalculate P matrix in its Positive Defenite variant
else
    disp('imaginary noise covariance matrix');return;
end

P_fixed_history(index+1,:) = diag(P(1:fixed_state_list_length,1:fixed_state_list_length));
P_att_history(:,:, index+1) = P(7:10, 7:10);
