%{
Inertial Navigation System Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

%}

set(0,'DefaultFigureWindowStyle','docked');
clear;close all;fclose all;clc;

%--> load data
load ('ems_simulated_trajectory.mat');
%load ('ems_real_data_trajectory.mat');

%--> Constants and Settings
R2D = 180/pi;
D2R = pi/180;
enable_animation = 0;
use_clean_imu = 0;
PROCESSING_DURATION_IN_SECS = 120;
N = length(ems_data.time);

%--> Initialization
sampling_period_sec = mean(diff(ems_data.time));
sampling_freq = round(1/sampling_period_sec);
NumRecords = PROCESSING_DURATION_IN_SECS/sampling_period_sec;
num_of_samples = NumRecords;

lat0 = ems_data.lat(1)*R2D;
lon0 = ems_data.lon(1)*R2D;
alt0 = ems_data.h(1);

%--> Add noise to the clean inertial sensor data (option)
if use_clean_imu == 0
    gyro_rw_stdv = 0.1; %deg/s
    gyro_bias = 0.25; %deg/s
    ems_data.gyro_noisy = ems_data.gyro_clean;
    ems_data.accel_noisy = ems_data.accel_clean;
    ems_data.gyro_noisy(:,3) = ems_data.gyro_clean(:,3) + ...
                               gyro_bias*D2R + ...
                               gyro_rw_stdv*D2R*randn(N,1);
end

%--> Initialize vectors sizes
time_vector_sec                         = ems_data.time(1:num_of_samples);
raw_acc_x                               = zeros(num_of_samples,1);
raw_acc_y                               = zeros(num_of_samples,1);
raw_acc_z                               = zeros(num_of_samples,1);
raw_gyro_x                              = zeros(num_of_samples,1);
raw_gyro_y                              = zeros(num_of_samples,1);
raw_gyro_z                              = zeros(num_of_samples,1);
quat_vector                             = zeros(num_of_samples,4);
Euler_pitch_value                       = zeros(num_of_samples,1);
Euler_roll_value                        = zeros(num_of_samples,1);
Euler_heading_value                     = zeros(num_of_samples,1);
ve_value                                = zeros(num_of_samples,1);
vn_value                                = zeros(num_of_samples,1);
vu_value                                = zeros(num_of_samples,1);
lat_value                               = zeros(num_of_samples,1);
lon_value                               = zeros(num_of_samples,1);
alt_value                               = zeros(num_of_samples,1);
Rn_value                                = zeros(num_of_samples,1);
Rm_value                                = zeros(num_of_samples,1);
EP_value                                = zeros(num_of_samples,1);
NP_value                                = zeros(num_of_samples,1);

gyro_bias_x_value                       = zeros(num_of_samples,1);
gyro_bias_y_value                       = zeros(num_of_samples,1);
gyro_bias_z_value                       = zeros(num_of_samples,1);
acc_bias_x_value                        = zeros(num_of_samples,1);
acc_bias_y_value                        = zeros(num_of_samples,1);
acc_bias_z_value                        = zeros(num_of_samples,1);

gyro_sf_x_value                         = zeros(num_of_samples,1);
gyro_sf_y_value                         = zeros(num_of_samples,1);
gyro_sf_z_value                         = zeros(num_of_samples,1);
acc_sf_x_value                          = zeros(num_of_samples,1);
acc_sf_y_value                          = zeros(num_of_samples,1);
acc_sf_z_value                          = zeros(num_of_samples,1);

g_value                                 = zeros(num_of_samples,1);

%--> Initialize the state
Euler_roll_value(1)     = ems_data.roll(1);
Euler_pitch_value(1)    = ems_data.pitch(1);
Euler_heading_value(1)  = ems_data.heading(1);
quat_vector(1,:) = angle2quat(Euler_heading_value(1),Euler_pitch_value(1),Euler_roll_value(1),'ZYX');
lat_value(1)  = ems_data.lat(1)*R2D;
lon_value(1)  = ems_data.lon(1)*R2D;
alt_value(1)  = ems_data.h(1);
EP_value(1) = 0;
NP_value(1) = 0;
g_value(1) = 9.8;

ve_value(1) = ems_data.vel_N(1,1);
vn_value(1) = ems_data.vel_N(1,2);
vu_value(1) = ems_data.vel_N(1,3);

%--> Initial biases
gyro_bias_x_value(1) = 0.0;
gyro_bias_y_value(1) = 0.0;
gyro_bias_z_value(1) = 0.0;
acc_bias_x_value(1)  = 0.0;
acc_bias_y_value(1)  = 0.0;
acc_bias_z_value(1)  = 0.0;

%--> Initial scale factors
gyro_sf_x_value(1) = 0.0;
gyro_sf_y_value(1) = 0.0;
gyro_sf_z_value(1) = 0.0;
acc_sf_x_value(1)  = 0.0;
acc_sf_y_value(1)  = 0.0;
acc_sf_z_value(1)  = 0.0;

%--> Main processing loop
for index = 1:NumRecords-1
    
    %--> Read raw IMU

    if use_clean_imu == 1
        raw_acc_x(index) = ems_data.accel_clean(index,1);
        raw_acc_y(index) = ems_data.accel_clean(index,2);
        raw_acc_z(index) = ems_data.accel_clean(index,3);
        raw_gyro_x(index) = ems_data.gyro_clean(index,1);
        raw_gyro_y(index) = ems_data.gyro_clean(index,2);
        raw_gyro_z(index) = ems_data.gyro_clean(index,3);
    else
        raw_acc_x(index) = ems_data.accel_noisy(index,1);
        raw_acc_y(index) = ems_data.accel_noisy(index,2);
        raw_acc_z(index) = ems_data.accel_noisy(index,3);
        raw_gyro_x(index) = ems_data.gyro_noisy(index,1);
        raw_gyro_y(index) = ems_data.gyro_noisy(index,2);
        raw_gyro_z(index) = ems_data.gyro_noisy(index,3);
    end

    acc = [raw_acc_x(index);raw_acc_y(index);raw_acc_z(index)];
    gyr = [raw_gyro_x(index);raw_gyro_y(index);raw_gyro_z(index)];
    r = [EP_value(index);NP_value(index);alt_value(index)];
    q = quat_vector(index,:)';
    v = [ve_value(index);vn_value(index);vu_value(index)];

    [r,v,q] = forward_kinematics(acc,gyr,r,v,q, lat_value(index)*D2R,sampling_period_sec);

    EP_value(index+1) = r(1);
    NP_value(index+1) = r(2);
    alt_value(index+1) = r(3);

    quat_vector(index + 1,:) = q;

    [Heading, pitch, roll] = quat2angle(q,'ZYX');
    
    Euler_roll_value(index + 1) = roll;
    Euler_pitch_value(index + 1) = pitch;
    Euler_heading_value(index + 1) = Heading;

    ve_value(index +1 ) = v(1);
    vn_value(index +1 ) = v(2);
    vu_value(index +1 ) = v(3);

    [lat,lon,alt] = enu2geodetic(r(1),r(2),r(3),lat0,lon0,alt0,wgs84Ellipsoid);

    lon_value(index+1) = lon;
    lat_value(index+1) = lat;
    
    %--> Advance biases
    gyro_bias_x_value(index+1) = gyro_bias_x_value(index);
    gyro_bias_y_value(index+1) = gyro_bias_y_value(index);
    gyro_bias_z_value(index+1) = gyro_bias_z_value(index);
    acc_bias_x_value(index+1) = acc_bias_x_value(index);
    acc_bias_y_value(index+1) = acc_bias_y_value(index);
    acc_bias_z_value(index+1) = acc_bias_z_value(index);
    
    %--> Advance scale factors
    gyro_sf_x_value(index+1) = gyro_sf_x_value(index);
    gyro_sf_y_value(index+1) = gyro_sf_y_value(index);
    gyro_sf_z_value(index+1) = gyro_sf_z_value(index);
    acc_sf_x_value(index+1) = acc_sf_x_value(index);
    acc_sf_y_value(index+1) = acc_sf_y_value(index);
    acc_sf_z_value(index+1) = acc_sf_z_value(index);
    g_value(index + 1) = 9.8;

    %--> Disply results in 3D
     if enable_animation == 1
        C_BL_value_plus_90 = angle2dcm(pi/2 - Euler_heading_value(index),-Euler_pitch_value(index),Euler_roll_value(index),'ZYX');
        C_LB_value_plus_90 = C_BL_value_plus_90';
          
        updated_vert = C_LB_value_plus_90*initial_vert';
        
        updated_vert = updated_vert';
        
        for p = 1:length(updated_vert)
            updated_vert(p,:) = updated_vert(p,:) + [EP_value(index), NP_value(index), alt_value(index)];
        end
        
        [ CubeXData , CubeYData , CubeZData ] = get_cube_axis_data(updated_vert);
        CubePoints = [updated_vert(:,1),updated_vert(:,2),updated_vert(:,3)];
     end

    fprintf('processing epoch %d/%d\n',index,num_of_samples);
end

%--> Plot error values
v_east_ref_vector       = ems_data.vel_N(1:NumRecords,1)';
v_north_ref_vector      = ems_data.vel_N(1:NumRecords,2)';
v_up_ref_vector         = ems_data.vel_N(1:NumRecords,3)';
p_east_ref_vector       = ems_data.east(1:NumRecords);
p_north_ref_vector      = ems_data.north(1:NumRecords);
alt_ref_vector          = ems_data.h(1:NumRecords);
roll_ref_vector         = ems_data.roll(1:NumRecords);
pitch_ref_vector        = ems_data.pitch(1:NumRecords);
heading_ref_vector      = ems_data.heading(1:NumRecords);

ve_error = v_east_ref_vector'-ve_value;
vn_error = v_north_ref_vector'-vn_value;
vu_error = v_up_ref_vector'-vu_value;
E_error = p_east_ref_vector-EP_value;
N_error = p_north_ref_vector-NP_value;
U_error = alt_ref_vector-alt_value;
roll_error = angdiff(roll_ref_vector,Euler_roll_value)*R2D;
pitch_error = angdiff(pitch_ref_vector,Euler_pitch_value)*R2D;
heading_error = angdiff(heading_ref_vector,Euler_heading_value)*R2D;

close all;
plot_processing_results;
plot_position_curves;
plot_error_curves;
print_error_values;