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

set(0,'DefaultFigureWindowStyle','docked');
close all;
fclose all;
clear;
clc;

enable_animation = 1;

D2R = pi/180;
R2D = 1/D2R;

%--> Load dataset and filter parameters
load('ems_IMU_pos');
load('ems_KITTI_dataset_seq07.mat');
load('points_100');     %feature points coordinates
load('features_100');   %feature points descriptors
load('ekf_params');

num_of_IMU_samples = length(dataset.time);
IMU_sampling_rate = 100;
IMU_sampling_period_sec = 0.01;
camera_frame_rate = 10;
GNSS_sampling_rate = 1;

image_set_size = length(features_all);

fixed_state_list_length = 37;

max_num_dtct_features = 100;

%--> Initialize the symbolic mechanization equation definition script
disp('Initialize the symbolic mechanization equation definition script....');
EMS_VIGO_SymbolicEngine;

%--> Converting symbolic functions into real-valued functions
C_LB_Euler_fun              = matlabFunction(C_LB_Euler);       
a_dot_fun                   = matlabFunction(a_dot);            
b_dot_fun                   = matlabFunction(b_dot);           
c_dot_fun                   = matlabFunction(c_dot);            
d_dot_fun                   = matlabFunction(d_dot);            
vel_dot_fun                 = matlabFunction(vel_dot);          
F_fixed_fun                 = matlabFunction(F_fixed);          
G_fixed_fun                 = matlabFunction(G_fixed);          
RM_fun                      = matlabFunction(RM);
RN_fun                      = matlabFunction(RN);               
r_fun                       = matlabFunction(r_j_i);            
U_fun                       = matlabFunction(U_f_j_i);          
J_r_fun                     = matlabFunction(J_r_j_i);          
H_cam_X_fixed_i_fun         = matlabFunction(H_cam_X_fixed_i);  
H_cam_X_pi_i_fun            = matlabFunction(H_cam_X_pi_i);     
J_Euler_Q_fun               = matlabFunction(J_Euler_Q);        

%--> Initialize vectors sizes
gyro_x_meas                 = zeros(num_of_IMU_samples,1);                   
gyro_y_meas                 = zeros(num_of_IMU_samples,1);
gyro_z_meas                 = zeros(num_of_IMU_samples,1);
gyro_x_bias_value           = zeros(num_of_IMU_samples,1);                  
gyro_y_bias_value           = zeros(num_of_IMU_samples,1);
gyro_z_bias_value           = zeros(num_of_IMU_samples,1);
gyro_x_scale_value          = ones(num_of_IMU_samples,1);                   
gyro_y_scale_value          = ones(num_of_IMU_samples,1);
gyro_z_scale_value          = ones(num_of_IMU_samples,1);

accel_x_meas                = zeros(num_of_IMU_samples,1);                 
accel_y_meas                = zeros(num_of_IMU_samples,1);
accel_z_meas                = zeros(num_of_IMU_samples,1);
accel_x_bias_value          = zeros(num_of_IMU_samples,1);                 
accel_y_bias_value          = zeros(num_of_IMU_samples,1);
accel_z_bias_value          = zeros(num_of_IMU_samples,1);
accel_x_scale_value         = ones(num_of_IMU_samples,1);                  
accel_y_scale_value         = ones(num_of_IMU_samples,1);
accel_z_scale_value         = ones(num_of_IMU_samples,1);

IMU_q_BI_value              = repmat([1,0,0,0], num_of_IMU_samples,1);      

vn_value                    = zeros(num_of_IMU_samples,1);                  
vn_value(1)                 = dataset.vel(1,1);
ve_value                    = zeros(num_of_IMU_samples,1);
ve_value(1)                 = dataset.vel(1,2);
vd_value                    = zeros(num_of_IMU_samples,1);
vd_value(1)                 = dataset.vel(1,3);
vu_value                    = zeros(num_of_IMU_samples,1);
vu_value(1)                 = -dataset.vel(1,3);

a_value                     = zeros(num_of_IMU_samples,1);                  
b_value                     = zeros(num_of_IMU_samples,1);
c_value                     = zeros(num_of_IMU_samples,1);
d_value                     = zeros(num_of_IMU_samples,1);
att_q_value                 = quatnormalize(angle2quat(dataset.heading(1),dataset.pitch(1),dataset.roll(1),'ZYX')); 
a_value(1)                  = att_q_value(1);
b_value(1)                  = att_q_value(2);
c_value(1)                  = att_q_value(3);
d_value(1)                  = att_q_value(4);
Euler_roll_value            = zeros(num_of_IMU_samples,1);                  
Euler_roll_value(1)         = dataset.roll(1);
Euler_pitch_value           = zeros(num_of_IMU_samples,1);
Euler_pitch_value(1)        = dataset.pitch(1);
Euler_heading_value         = zeros(num_of_IMU_samples,1);
Euler_heading_value(1)      = dataset.heading(1);

pos_lat_value               = zeros(num_of_IMU_samples,1);
pos_lat_value(1)            = dataset.lat(1)*R2D;
pos_lon_value               = zeros(num_of_IMU_samples,1);
pos_lon_value(1)            = dataset.lon(1)*R2D;
pos_alt_value               = zeros(num_of_IMU_samples,1);
pos_alt_value(1)            = dataset.alt(1);                               
pos_north_value             = zeros(num_of_IMU_samples,1);                  
pos_north_value(1)          = dataset.north(1);                      
pos_east_value              = zeros(num_of_IMU_samples,1);
pos_east_value(1)           = dataset.east(1);                     
pos_down_value              = zeros(num_of_IMU_samples,1);
pos_down_value(1)           = -dataset.alt(1);

cam_ints_value              = zeros(num_of_IMU_samples,4);                  
cam_ints_value(1,:)         = [dataset.K(1,1),dataset.K(2,2),dataset.K(1,3)+1,dataset.K(2,3)+1];    

cam_exts_value              = zeros(num_of_IMU_samples, 7);                      
cam_exts_value(1, 1:4)      = rotm2quat(dataset.IMU2Cam(1:3,1:3)');         
cam_exts_value(1, 5:7)      = (dataset.IMU2Cam(1:3,1:3)')*(-dataset.IMU2Cam(1:3,4));    

cam_pos_value               = zeros(num_of_IMU_samples, 3);                 
cam_pos_value(1, :)         = [pos_north_value(1), pos_east_value(1), pos_down_value(1)]+... 
                              (quat2rotm([a_value(1) b_value(1) c_value(1) d_value(1)])*cam_exts_value(1, 5:7)')';
                          
cam_att_value               = zeros(num_of_IMU_samples, 4);                 
cam_att_value(1, :)         = quatnormalize(quatmultiply([a_value(1) b_value(1) c_value(1) d_value(1)], cam_exts_value(1, 1:4)));

cam_roll_value              = zeros(num_of_IMU_samples,1);                  
cam_pitch_value             = zeros(num_of_IMU_samples,1);
cam_heading_value           = zeros(num_of_IMU_samples,1);
[A, p, r]                   = quat2angle([cam_att_value(1, 1), cam_att_value(1, 2),cam_att_value(1, 3),cam_att_value(1, 4)],'ZYX');
cam_roll_value(1)           = r;
cam_pitch_value(1)          = p;
cam_heading_value(1)        = A;

tracking_pose_indices       = [];

update_feature_size         = zeros(num_of_IMU_samples/camera_frame_rate,1);

map_features_count          = 1;
map_features_pos            = cell(1, 1);
map_features_pos{1}         = single([0,0, -cam_pos_value(1, 3)]);

P_fixed_history             = zeros(num_of_IMU_samples,fixed_state_list_length);
P_att_history               = zeros(4,4, num_of_IMU_samples);

frame_id                    = 0;

wg_noise_value              = 0.0;

v_north_ref_vector      = dataset.vel(:,1)';
v_east_ref_vector       = dataset.vel(:,2)';
v_down_ref_vector       = dataset.vel(:,3)';
p_north_ref_vector      = dataset.north';
p_east_ref_vector       = dataset.east';
p_down_ref_vector       = -dataset.alt';
alt_ref_vector          = dataset.alt';
roll_ref_vector         = dataset.roll';
pitch_ref_vector        = dataset.pitch';
heading_ref_vector      = dataset.heading';

%--> IMU noise parameters
gyro_x_std_value = parameters(1);
gyro_y_std_value = parameters(2);
gyro_z_std_value = parameters(3);
gyro_x_bias_std_value = parameters(4);
gyro_y_bias_std_value = parameters(5);
gyro_z_bias_std_value = parameters(6);
gyro_x_scale_std_value = parameters(7);
gyro_y_scale_std_value = parameters(8);
gyro_z_scale_std_value = parameters(9);

accel_x_std_value = parameters(10);
accel_y_std_value = parameters(11);
accel_z_std_value = parameters(12);
accel_x_bias_std_value = parameters(13);
accel_y_bias_std_value = parameters(14);
accel_z_bias_std_value = parameters(15);
accel_x_scale_std_value = parameters(16);
accel_y_scale_std_value = parameters(17);
accel_z_scale_std_value = parameters(18);

%--> Camera noise parameters
pxl_std_value = parameters(19);
furthest_depth = parameters(20);
max_depth_std = parameters(21);
chi_perc = parameters(22);

%--> Initial error states covariances
pos_north_error_covariance = parameters(23);
pos_east_error_covariance = parameters(24);
pos_down_error_covariance = parameters(25);
vn_error_covariance = parameters(26);
ve_error_covariance = parameters(27);
vd_error_covariance = parameters(28);
a_error_covariance = parameters(29);
b_error_covariance = parameters(30);
c_error_covariance = parameters(31);
d_error_covariance = parameters(32);
gyro_x_bias_error_covariance = parameters(33);
gyro_y_bias_error_covariance = parameters(34);
gyro_z_bias_error_covariance = parameters(35);
accel_x_bias_error_covariance = parameters(36);
accel_y_bias_error_covariance = parameters(37);
accel_z_bias_error_covariance = parameters(38);
gyro_x_scale_error_covariance = parameters(39);
gyro_y_scale_error_covariance = parameters(40);
gyro_z_scale_error_covariance = parameters(41);
accel_x_scale_error_covariance = parameters(42);
accel_y_scale_error_covariance = parameters(43);
accel_z_scale_error_covariance = parameters(44);
IMU_calib_a_error_covariance = parameters(45);
IMU_calib_b_error_covariance = parameters(46);
IMU_calib_c_error_covariance = parameters(47);
IMU_calib_d_error_covariance = parameters(48);
cam_ints_fx_error_covariance = parameters(49);
cam_ints_fy_error_covariance = parameters(50);
cam_ints_ox_error_covariance = parameters(51);
cam_ints_oy_error_covariance = parameters(52);
cam_exts_a_error_covariance = parameters(53);
cam_exts_b_error_covariance = parameters(54);
cam_exts_c_error_covariance = parameters(55);
cam_exts_d_error_covariance = parameters(56);
cam_exts_x_error_covariance = parameters(57);
cam_exts_y_error_covariance = parameters(58);
cam_exts_z_error_covariance = parameters(59);

P_fixed = diag([pos_north_error_covariance; pos_east_error_covariance; pos_down_error_covariance;
                vn_error_covariance; ve_error_covariance; vd_error_covariance;
                a_error_covariance; b_error_covariance; c_error_covariance; d_error_covariance;
                gyro_x_bias_error_covariance; gyro_y_bias_error_covariance; gyro_z_bias_error_covariance;
                accel_x_bias_error_covariance; accel_y_bias_error_covariance; accel_z_bias_error_covariance;
                gyro_x_scale_error_covariance; gyro_y_scale_error_covariance; gyro_z_scale_error_covariance;
                accel_x_scale_error_covariance; accel_y_scale_error_covariance; accel_z_scale_error_covariance;  
                IMU_calib_a_error_covariance; IMU_calib_b_error_covariance; IMU_calib_c_error_covariance; IMU_calib_d_error_covariance;
                cam_ints_fx_error_covariance; cam_ints_fy_error_covariance; cam_ints_ox_error_covariance; cam_ints_oy_error_covariance;
                cam_exts_a_error_covariance; cam_exts_b_error_covariance; cam_exts_c_error_covariance; cam_exts_d_error_covariance;
                cam_exts_x_error_covariance; cam_exts_y_error_covariance; cam_exts_z_error_covariance]);

%--> Initialization of the the complete covariance matrix
P = P_fixed;        
P_fixed_history(1,:) = diag(P_fixed);
P_att_history(:,:,1) = P_fixed(7:10, 7:10);

%--> Draw a 3D Object for animation
X = 12;Y = 3;Z = 1;
origin = [X/2 Y/2 Z/2];
initial_vert = [X Y 0;              %(1)
                0 Y 0;              %(2)
                0 Y Z;              %(3)
                X Y Z;              %(4)
                0 0 Z;              %(5)
                X 0 Z;              %(6)
                X 0 0;              %(7)
                0 0 0;              %(8)
                1.3*X Y/2 0;        %(9)
                1.3*X Y/2 0.5*Z;    %(10)
];
for p = 1:length(initial_vert)
    initial_vert(p,:) = initial_vert(p,:) - origin;
end
[ CubeXData , CubeYData , CubeZData ] = get_cube_axis_data(initial_vert);
faces = [1 2 3 4; 4 3 5 6; 6 7 8 5; 1 2 8 7; 6 7 1 4; 2 3 5 8;1 9 10 4;4 10 6 6;6 10 9 7;1 9 7 7];

%--> Main processing loop
disp('Main processing loop....');

end_index = length(dataset.time)-1;

for index = 1:end_index

    fprintf('Processing epoch %d/%d\n',index,end_index);

    IMU_sampling_period_sec = abs(diff(dataset.time(index:index+1)));
    
    %--> Predict states using inertial data x_k+1 = f(x_k,u_k)
    propagate_states;
        
    %--> Predict error covariances P_k+1 = Phi_k*P_k*Phi_k' + Q_k
    propagate_covariances;
    
    %--> Apply vision updates using visual features
    if (mod(index,round(IMU_sampling_rate/camera_frame_rate)) == 1)        
        vision_update;        
    end

    %--> Calculate the errors
    if (mod(index,100) == 1) 
        Ne = sqrt(mean((p_north_ref_vector(1:camera_frame_rate:index)'-pos_north_value(1:camera_frame_rate:index)).^2));
        Ee = sqrt(mean((p_east_ref_vector(1:camera_frame_rate:index)'-pos_east_value(1:camera_frame_rate:index)).^2));
        Ae = sqrt(mean((alt_ref_vector(1:camera_frame_rate:index)'-pos_alt_value(1:camera_frame_rate:index)).^2));
        Vne = sqrt(mean((v_north_ref_vector(1:camera_frame_rate:index)'-vn_value(1:camera_frame_rate:index)).^2));
        Vee = sqrt(mean((v_east_ref_vector(1:camera_frame_rate:index)'-ve_value(1:camera_frame_rate:index)).^2));
        Vde = sqrt(mean((v_down_ref_vector(1:camera_frame_rate:index)'-vd_value(1:camera_frame_rate:index)).^2));
        Re = sqrt(mean(atan(tan((roll_ref_vector (1:camera_frame_rate:index)'-Euler_roll_value(1:camera_frame_rate:index)))).^2))*R2D;
        Pe = sqrt(mean(atan(tan((pitch_ref_vector(1:camera_frame_rate:index)'-Euler_pitch_value(1:camera_frame_rate:index)))).^2))*R2D;
        He = sqrt(mean(atan(tan((heading_ref_vector(1:camera_frame_rate:index)'-Euler_heading_value(1:camera_frame_rate:index)))).^2))*R2D;        
    end
    
    %--> Disply results in 3D
    display_vehicle_in_3D;
    map_features_pos_mat = cell2mat(map_features_pos);
    
    %--> Create initial figures
    if(index == 1) 
        create_figures;
    else
        if enable_animation == 1
            if (mod(index,100) == 1)
                refreshdata(h_ekf_position, 'caller');
                refreshdata(h_map, 'caller'); 
                refreshdata(h_cov, 'caller');
                set(h_vehicle,'XData',CubeXData,'YData',CubeYData,'ZData',CubeZData);
                drawnow
            end
        end
    end
end

show_results;