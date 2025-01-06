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

clc;

%--> Print error values
north_error_vector = p_north_ref_vector(1:end_index)'-pos_north_value(1:end_index);
east_error_vector = p_east_ref_vector(1:end_index)'-pos_east_value(1:end_index);
alt_error_vector = alt_ref_vector(1:end_index)'-pos_alt_value(1:end_index);
v_error_north_vector = v_north_ref_vector(1:end_index)'-vn_value(1:end_index);
v_error_east_vector = v_east_ref_vector(1:end_index)'-ve_value(1:end_index);
v_error_down_vector = v_down_ref_vector(1:end_index)'-vd_value(1:end_index);
roll_error_vector = angdiff(roll_ref_vector (1:end_index)',Euler_roll_value(1:end_index))*R2D;
pitch_error_vector = angdiff(pitch_ref_vector (1:end_index)',Euler_pitch_value(1:end_index))*R2D;
heading_error_vector = angdiff(heading_ref_vector (1:end_index)',Euler_heading_value(1:end_index))*R2D;

Ne = sqrt(mean(north_error_vector.^2));
Ee = sqrt(mean(east_error_vector.^2));
Ae = sqrt(mean(alt_error_vector.^2));

Vne = sqrt(mean(v_error_north_vector.^2));
Vee = sqrt(mean(v_error_east_vector.^2));
Vde = sqrt(mean(v_error_down_vector.^2));

Re = sqrt(mean(roll_error_vector.^2));
Pe = sqrt(mean(pitch_error_vector.^2));
He = sqrt(mean(heading_error_vector.^2));

fprintf('\n\nError mean whole trajectory at 100 Hz\n');
fprintf('North error(m) = %.4f\n', Ne);
fprintf('East error(m) = %.4f\n', Ee);
fprintf('Alt error(m) = %.4f\n', Ae);
fprintf('2D error(m) = %.4f\n', sqrt(Ne^2+Ee^2));
fprintf('V north error(m/s) = %.4f\n',Vne);
fprintf('V east error(m/s) = %.4f\n', Vee);
fprintf('V down error(m/s) = %.4f\n', Vde);
fprintf('Roll error(deg) = %.4f\n', Re);
fprintf('Pitch error(deg) = %.4f\n', Pe);
fprintf('Heading error(deg) = %.4f\n', He);

traj_length = sum(sqrt(diff(p_north_ref_vector(1:camera_frame_rate:end_index)).^2 + diff(p_east_ref_vector(1:camera_frame_rate:end_index)).^2));
north_relative_diff = diff(p_north_ref_vector(1:camera_frame_rate:end_index)') - diff(pos_north_value(1:camera_frame_rate:end_index));
east_relative_diff = diff(p_east_ref_vector(1:camera_frame_rate:end_index)') - diff(pos_east_value(1:camera_frame_rate:end_index));
alt_relative_diff = diff(alt_ref_vector(1:camera_frame_rate:end_index)') - diff(pos_alt_value(1:camera_frame_rate:end_index));
translation_3D_error = sqrt(north_relative_diff.^2 + east_relative_diff.^2 + alt_relative_diff.^2);
translation_2D_error = sqrt(north_relative_diff.^2 + east_relative_diff.^2);

roll_relative_diff =  R2D.*(diff(roll_ref_vector (1:camera_frame_rate:end_index)') - diff(Euler_roll_value(1:camera_frame_rate:end_index)));
pitch_relative_diff =  R2D.*(diff(pitch_ref_vector(1:camera_frame_rate:end_index)') - diff(Euler_pitch_value(1:camera_frame_rate:end_index)));
heading_relative_diff =  R2D.*(diff(heading_ref_vector(1:camera_frame_rate:end_index)') - diff(Euler_heading_value(1:camera_frame_rate:end_index)));
rotation_3D_error = sqrt(roll_relative_diff.^2 + pitch_relative_diff.^2 + heading_relative_diff.^2);

fprintf('\nKITTI Metric Error 10 Hz:');
fprintf('\nTrajectory Length(m) = %.4f', traj_length);
fprintf('\nTrajectory Duration(s) = %d\n', floor(dataset.time(end)-dataset.time(1)));
fprintf('\nNorth translation error(m) = %.4f\n', sum(abs(north_relative_diff))/(image_set_size));
fprintf('North translation error(%%) = %.4f\n', sum(abs(north_relative_diff))/(traj_length*image_set_size));
fprintf('\nEast translation error(m) = %.4f\n', sum(abs(east_relative_diff))/(image_set_size));
fprintf('East translation error(%%) = %.4f\n',sum(abs(east_relative_diff))/(traj_length*image_set_size));
fprintf('\nAlt translation error(m) = %.4f\n', sum(abs(alt_relative_diff))/(image_set_size));
fprintf('Alt translation error(%%) = %.4f\n', sum(abs(alt_relative_diff))/(traj_length*image_set_size));
fprintf('\n2D translation error(m) = %.4f\n', sum(translation_2D_error)/(image_set_size));
fprintf('2D translation error(%%) = %.4f\n', sum(translation_2D_error)/(traj_length*image_set_size));
fprintf('\n3D translation error(m) = %.4f\n', sum(translation_3D_error)/(image_set_size));
fprintf('3D translation error(%%) = %.4f\n', sum(translation_3D_error)/(traj_length*image_set_size));
fprintf('\nRoll rotation error(deg) = %.4f\n', sum(abs(roll_relative_diff))/(image_set_size));
fprintf('Roll rotation error(deg/m) = %.4f\n', sum(abs(roll_relative_diff))/(traj_length*image_set_size));
fprintf('\nPitch rotation error(deg) = %.4f\n', sum(abs(pitch_relative_diff))/(image_set_size));
fprintf('Pitch rotation error(deg/m) = %.4f\n', sum(abs(pitch_relative_diff))/(traj_length*image_set_size));
fprintf('\nHeading rotation error(deg) = %.4f\n', sum(abs(heading_relative_diff))/(image_set_size));
fprintf('Heading rotation error(deg/m) = %.4f\n', sum(abs(heading_relative_diff))/(traj_length*image_set_size));
fprintf('\nFull rotation error(deg) = %.4f\n', sum(rotation_3D_error)/(image_set_size));
fprintf('Full rotation error(deg/m) = %.4f\n', sum(rotation_3D_error)/(traj_length*image_set_size));

figure;title('Position Errors');
subplot(3,1,1);
Vertices(:,2) = [north_error_vector + 1*sqrt(P_fixed_history(1:end-1, 1)); flipdim(north_error_vector - 1*sqrt(P_fixed_history(1:end-1, 1)),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), north_error_vector,'b'); hold on;
title('North Error'); grid on; xlabel('Time (s)');ylabel('Error (m)');  legend('\pm2\sigma','Error');

subplot(3,1,2);
Vertices(:,2) = [east_error_vector + 1*sqrt(P_fixed_history(1:end-1, 2)); flipdim(east_error_vector - 1*sqrt(P_fixed_history(1:end-1, 2)),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), east_error_vector,'b'); hold on;
title('East Error'); grid on; xlabel('Time (s)');ylabel('Error (m)');  legend('\pm2\sigma','Error');

subplot(3,1,3);
Vertices(:,2) = [alt_error_vector + 1*sqrt(P_fixed_history(1:end-1, 3)); flipdim(alt_error_vector - 1*sqrt(P_fixed_history(1:end-1, 3)),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), alt_error_vector,'b'); hold on;
title('East Error'); grid on; xlabel('Time (s)');ylabel('Error (m)');  legend('\pm2\sigma','Error');

figure;title('Velocity Errors');
subplot(3,1,1);
Vertices(:,2) = [v_error_north_vector + 1*sqrt(P_fixed_history(1:end-1, 4)); flipdim(v_error_north_vector - 1*sqrt(P_fixed_history(1:end-1, 4)),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), v_error_north_vector,'b'); hold on;
title('North Velocity Error'); grid on; xlabel('Time (s)');ylabel('Error (m/s)');  legend('\pm2\sigma','Error');

subplot(3,1,2);
Vertices(:,2) = [v_error_east_vector + 1*sqrt(P_fixed_history(1:end-1, 5)); flipdim(v_error_east_vector - 1*sqrt(P_fixed_history(1:end-1, 5)),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), v_error_east_vector,'b'); hold on;
title('East Velocity Error'); grid on; xlabel('Time (s)');ylabel('Error (m/s)');  legend('\pm2\sigma','Error');

subplot(3,1,3);
Vertices(:,2) = [v_error_down_vector + 1*sqrt(P_fixed_history(1:end-1, 6)); flipdim(v_error_down_vector - 1*sqrt(P_fixed_history(1:end-1, 6)),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), v_error_down_vector,'b'); hold on;
title('Down Velocity Error'); grid on; xlabel('Time (s)');ylabel('Error (m/s)');  legend('\pm2\sigma','Error');

Euler_roll_error_std = zeros(num_of_IMU_samples ,1);
Euler_pitch_error_std = zeros(num_of_IMU_samples ,1);
Euler_heading_error_std = zeros(num_of_IMU_samples ,1);

for i=1:num_of_IMU_samples
    Q = angle2quat(Euler_heading_value(i),...
                   Euler_pitch_value(i),...
                   Euler_roll_value(i),'ZYX');
    J_E_q = J_Euler_Q_fun(Q(1), Q(2), Q(3), Q(4));
    cov_q = P_att_history(:,:,i);

    
    cov_angle = J_E_q*cov_q*J_E_q';   
    Euler_roll_error_std(i) = sqrt(cov_angle(1,1));
    Euler_pitch_error_std(i) = sqrt(cov_angle(2,2));
    Euler_heading_error_std(i) = sqrt(cov_angle(3,3));
end

figure;title('Attitude Error');
subplot(3,1,1);
Vertices(:,2) = [roll_error_vector + 1*sqrt(Euler_roll_error_std(1:end-1)*R2D); flipdim(roll_error_vector - 1*sqrt(Euler_roll_error_std(1:end-1)*R2D),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), roll_error_vector,'b'); hold on;
title('Roll Error'); grid on; xlabel('Time (s)');ylabel('Error (deg)');  legend('\pm2\sigma','Error');

subplot(3,1,2);
Vertices(:,2) = [pitch_error_vector + 1*sqrt(Euler_pitch_error_std(1:end-1)*R2D); flipdim(pitch_error_vector - 1*sqrt(Euler_pitch_error_std(1:end-1)*R2D),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), pitch_error_vector,'b'); hold on;
title('Pitch Error'); grid on; xlabel('Time (s)');ylabel('Error (deg)');  legend('\pm2\sigma','Error');

subplot(3,1,3);
Vertices(:,2) = [heading_error_vector + 1*sqrt(Euler_heading_error_std(1:end-1)*R2D); flipdim(heading_error_vector - 1*sqrt(Euler_heading_error_std(1:end-1)*R2D),1)];
Vertices(:,1) = [dataset.time(1:end-1)-dataset.time(1);flipdim(dataset.time(1:end-1)-dataset.time(1),1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(dataset.time(1:end-1)-dataset.time(1), heading_error_vector,'b'); hold on;
title('Heading Error'); grid on; xlabel('Time (s)');ylabel('Error (deg)');  legend('\pm2\sigma','Error');

fprintf('\nTotal number of map feature points = %d\n', sum(update_feature_size));