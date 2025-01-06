%{
Imu Gps Fusion Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

set(0,'DefaultFigureWindowStyle','docked');
clear;
close all;
fclose all;
clc;

%--> Load data
%load ('short_simulated_trajectory.mat');
%load ('long_simulated_trajectory.mat');
load ('real_trajectory_1.mat');
%load ('real_trajectory_2.mat');

%--> General Settings
rng(0);
R2D = 180/pi;
use_clean_imu = 0;
forward_kinematics_only = 0;
use_forward_knematics_function = 1;
num_of_samples = length(ems_data.time);
sampling_period_sec = mean(diff(ems_data.time));
sampling_freq = round(1/sampling_period_sec);

%--> Apply Inverse Kinematics with Known Noise Parameters to simulate consumer grdade IMU noise
gyroNoise = [0.01 0.01 0.01]*sqrt(sampling_freq);
gyroBias = [0.2 0.4 0.6];
gyroSF = [0 0 0];
accelNoise = [0.0029 0.0029 0.0029]*sqrt(sampling_freq);
accelBias = [0.1 0.2 .3];
accelSF = [0 0 0];
posNoise = [1 1 1];
velNoise = [0.01 0.01 0.01];
rangeNoise = 0;
rangeRateNoise =0;
ems_data = InverseKinematics(ems_data,gyroNoise, gyroBias, gyroSF, accelNoise, accelBias, accelSF, posNoise, velNoise);

%--> Display Raw IMU Data
figure;
subplot(3,2,1);
plot(ems_data.time,ems_data.gyro_clean(:,1)*R2D,'b');hold on;grid on;xlabel('time(s)');
plot(ems_data.time,ems_data.gyro_noisy(:,1)*R2D,'r');title('gyro x(deg)');legend('clean','noisy');
subplot(3,2,3);
plot(ems_data.time,ems_data.gyro_clean(:,2)*R2D,'b');hold on;grid on;xlabel('time(s)');
plot(ems_data.time,ems_data.gyro_noisy(:,2)*R2D,'r');title('gyro y(deg)');legend('clean','noisy');
subplot(3,2,5);
plot(ems_data.time,ems_data.gyro_clean(:,3)*R2D,'b');hold on;grid on;xlabel('time(s)');
plot(ems_data.time,ems_data.gyro_noisy(:,3)*R2D,'r');title('gyro z(deg)');legend('clean','noisy');
subplot(3,2,2);
plot(ems_data.time,ems_data.accel_clean(:,1),'b');hold on;grid on;xlabel('time(s)');
plot(ems_data.time,ems_data.accel_noisy(:,1),'r');title('accel x(m/s2)');legend('clean','noisy');
subplot(3,2,4);
plot(ems_data.time,ems_data.accel_clean(:,2),'b');hold on;grid on;xlabel('time(s)');
plot(ems_data.time,ems_data.accel_noisy(:,2),'r');title('accel y(m/s2)');legend('clean','noisy');
subplot(3,2,6);
plot(ems_data.time,ems_data.accel_clean(:,3),'b');hold on;grid on;xlabel('time(s)');
plot(ems_data.time,ems_data.accel_noisy(:,3),'r');title('accel z(m/s2)');legend('clean','noisy');

%--> Call symbolic mechanization and error equation definition script
ems_symbolic_engine;

%--> Converting symbolic functions into real-valued functions
F_fun                       = matlabFunction(F);
G_fun                       = matlabFunction(G);

%--> Earth ellipsoid shape parameters
earth_a           = 6378137;
earth_f           = 1/298.257223563;
earth_b           = earth_a*(1-earth_f);
earth_e2          = 1-(earth_b^2)/(earth_a^2);
we_value          = 2*pi/(24*60*60);

%--> Initialization
time_vector_sec                         = ems_data.time;
raw_acc_x                               = zeros(num_of_samples,1);
raw_acc_y                               = zeros(num_of_samples,1);
raw_acc_z                               = zeros(num_of_samples,1);
raw_gyro_x                              = zeros(num_of_samples,1);
raw_gyro_y                              = zeros(num_of_samples,1);
raw_gyro_z                              = zeros(num_of_samples,1);
a_value                                 = zeros(num_of_samples,1);
b_value                                 = zeros(num_of_samples,1);
c_value                                 = zeros(num_of_samples,1);
d_value                                 = zeros(num_of_samples,1);
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

gyro_rw_stdv_x_value                    = zeros(num_of_samples,1);
gyro_rw_stdv_y_value                    = zeros(num_of_samples,1);
gyro_rw_stdv_z_value                    = zeros(num_of_samples,1);
gyro_bias_gauss_markov_stdv_x_value     = zeros(num_of_samples,1);
gyro_bias_gauss_markov_stdv_y_value     = zeros(num_of_samples,1);
gyro_bias_gauss_markov_stdv_z_value     = zeros(num_of_samples,1);
gyro_bias_x_time_cnst_value             = zeros(num_of_samples,1);
gyro_bias_y_time_cnst_value             = zeros(num_of_samples,1);
gyro_bias_z_time_cnst_value             = zeros(num_of_samples,1);
gyro_sf_gauss_markov_stdv_x_value       = zeros(num_of_samples,1);
gyro_sf_gauss_markov_stdv_y_value       = zeros(num_of_samples,1);
gyro_sf_gauss_markov_stdv_z_value       = zeros(num_of_samples,1);
gyro_sf_x_time_cnst_value               = zeros(num_of_samples,1);
gyro_sf_y_time_cnst_value               = zeros(num_of_samples,1);
gyro_sf_z_time_cnst_value               = zeros(num_of_samples,1);

acc_rw_stdv_x_value                     = zeros(num_of_samples,1);
acc_rw_stdv_y_value                     = zeros(num_of_samples,1);
acc_rw_stdv_z_value                     = zeros(num_of_samples,1);
acc_bias_gauss_markov_stdv_x_value      = zeros(num_of_samples,1);
acc_bias_gauss_markov_stdv_y_value      = zeros(num_of_samples,1);
acc_bias_gauss_markov_stdv_z_value      = zeros(num_of_samples,1);
acc_bias_x_time_cnst_value              = zeros(num_of_samples,1);
acc_bias_y_time_cnst_value              = zeros(num_of_samples,1);
acc_bias_z_time_cnst_value              = zeros(num_of_samples,1);
acc_sf_gauss_markov_stdv_x_value        = zeros(num_of_samples,1);
acc_sf_gauss_markov_stdv_y_value        = zeros(num_of_samples,1);
acc_sf_gauss_markov_stdv_z_value        = zeros(num_of_samples,1);
acc_sf_x_time_cnst_value                = zeros(num_of_samples,1);
acc_sf_y_time_cnst_value                = zeros(num_of_samples,1);
acc_sf_z_time_cnst_value                = zeros(num_of_samples,1);

alt_rw_stdv_value                       = zeros(num_of_samples,1);

wg_noise_value                          = zeros(num_of_samples,1);

%--> Initialize the state
lat0 = ems_data.lat(1);
lon0 = ems_data.lon(1);
alt0 = ems_data.h(1);

Euler_roll_value(1)     = ems_data.roll(1);
Euler_pitch_value(1)    = ems_data.pitch(1);
Euler_heading_value(1)  = ems_data.heading(1);
attitude_quat_value = angle2quat(Euler_heading_value(1),Euler_pitch_value(1),Euler_roll_value(1),'ZYX');
a_value(1) = attitude_quat_value(1);
b_value(1) = attitude_quat_value(2);
c_value(1) = attitude_quat_value(3);
d_value(1) = attitude_quat_value(4);
lat_value(1)  = ems_data.lat(1)*R2D;
lon_value(1)  = ems_data.lon(1)*R2D;
alt_value(1)  = ems_data.h(1);
Rn_value(1)   = earth_a/sqrt(1-earth_e2*sin(lat_value(1)*D2R)*sin(lat_value(1)*D2R));
Rm_value(1)   = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat_value(1)*D2R)*sin(lat_value(1)*D2R))^(1.5));
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

%--> System noise parameters
alt_rw_stdv_value(1)    = 0.0;
acc_rw_stdv_x_value(1) = 0.01;
acc_rw_stdv_y_value(1) = 0.01;
acc_rw_stdv_z_value(1) = 0.01;
acc_bias_gauss_markov_stdv_x_value (1) = 0.01;
acc_bias_gauss_markov_stdv_y_value (1) = 0.01;
acc_bias_gauss_markov_stdv_z_value (1) = 0.01;
acc_bias_x_time_cnst_value(1) = 3*3600;
acc_bias_y_time_cnst_value(1) = 3*3600;
acc_bias_z_time_cnst_value(1) = 3*3600;
acc_sf_gauss_markov_stdv_x_value (1) = 1e-12;
acc_sf_gauss_markov_stdv_y_value (1) = 1e-12;
acc_sf_gauss_markov_stdv_z_value (1) = 1e-12;
acc_sf_x_time_cnst_value(1) = 3*3600;
acc_sf_y_time_cnst_value(1) = 3*3600;
acc_sf_z_time_cnst_value(1) = 3*3600;
gyro_rw_stdv_x_value(1) = 0.01;
gyro_rw_stdv_y_value(1) = 0.01;
gyro_rw_stdv_z_value(1) = 0.01;
gyro_bias_gauss_markov_stdv_x_value (1) = 0.001*D2R;
gyro_bias_gauss_markov_stdv_y_value (1) = 0.001*D2R;
gyro_bias_gauss_markov_stdv_z_value (1) = 0.001*D2R;
gyro_bias_x_time_cnst_value(1) = 3*3600;
gyro_bias_y_time_cnst_value(1) = 3*3600;
gyro_bias_z_time_cnst_value(1) = 3*3600;
gyro_sf_gauss_markov_stdv_x_value (1) = 1e-12;
gyro_sf_gauss_markov_stdv_y_value (1) = 1e-12;
gyro_sf_gauss_markov_stdv_z_value (1) = 1e-12;
gyro_sf_x_time_cnst_value(1) = 3*3600;
gyro_sf_y_time_cnst_value(1) = 3*3600;
gyro_sf_z_time_cnst_value(1) = 3*3600;

%--> Noise is to set to zero when applying state prediction using IMU sensors
wg_noise_value(1) = 0.0;

%--> Initial error states covariances
east_pos_error_covariance(1) = 25^2;
north_pos_error_covariance(1) = 25^2;
alt_error_covariance(1) = 20^2;
ve_error_covariance(1) = 50.05^2;
vn_error_covariance(1) = 50.05^2;
vu_error_covariance(1) = 50.05^2;
b_error_covariance(1) = 2.5^2;
c_error_covariance(1) = 2.5^2;
d_error_covariance(1) = 2.5^2;
gyro_bias_x_error_covariance(1) = .2^2;
gyro_bias_y_error_covariance(1) = .4^2;
gyro_bias_z_error_covariance(1) = .6^2;
acc_bias_x_error_covariance(1) = 0.1^2;
acc_bias_y_error_covariance(1) = 0.1^2;
acc_bias_z_error_covariance(1) = 0.05^2;
gyro_sf_x_error_covariance(1) = 0.001^2;
gyro_sf_y_error_covariance(1) = 0.001^2;
gyro_sf_z_error_covariance(1) = 0.001^2;
acc_sf_x_error_covariance(1) = 0.001^2;
acc_sf_y_error_covariance(1) = 0.001^2;
acc_sf_z_error_covariance(1) = 0.001^2;

P = diag([
    east_pos_error_covariance(1);
    north_pos_error_covariance(1);
    alt_error_covariance(1);
    ve_error_covariance(1);
    vn_error_covariance(1);
    vu_error_covariance(1);
    b_error_covariance(1);
    c_error_covariance(1);
    d_error_covariance(1);
    gyro_bias_x_error_covariance(1);
    gyro_bias_y_error_covariance(1);
    gyro_bias_z_error_covariance(1);
    acc_bias_x_error_covariance(1);
    acc_bias_y_error_covariance(1);
    acc_bias_z_error_covariance(1);
    gyro_sf_x_error_covariance(1);
    gyro_sf_y_error_covariance(1);
    gyro_sf_z_error_covariance(1);
    acc_sf_x_error_covariance(1);
    acc_sf_y_error_covariance(1);
    acc_sf_z_error_covariance(1)
    ]);

%--> Main processing loop
for index = 1:length(ems_data.time)-1

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

    %--> Apply Forward Kinematics (INS dynamic prediction motion equations)
    
    if use_forward_knematics_function == 1

        acc = [raw_acc_x(index)*(1+acc_sf_x_value(index))-acc_bias_x_value(index);
               raw_acc_y(index)*(1+acc_sf_y_value(index))-acc_bias_y_value(index);
               raw_acc_z(index)*(1+acc_sf_z_value(index))-acc_bias_z_value(index)];

        gyr = [raw_gyro_x(index)*(1+gyro_sf_x_value(index))-gyro_bias_x_value(index);
               raw_gyro_y(index)*(1+gyro_sf_y_value(index))-gyro_bias_y_value(index);
               raw_gyro_z(index)*(1+gyro_sf_z_value(index))-gyro_bias_z_value(index)];        

        r = [EP_value(index);NP_value(index);alt_value(index)];
        q = [a_value(index);b_value(index);c_value(index);d_value(index)];
        v = [ve_value(index);vn_value(index);vu_value(index)];
        lat = lat_value(index)*D2R;
        lon = lon_value(index)*D2R;

        [r,v,q,lat,lon] = forward_kinematics(acc,gyr,r,v,q,lat0,lon0,lat,lon,sampling_period_sec);
    
        EP_value(index+1) = r(1);
        NP_value(index+1) = r(2);
        alt_value(index+1) = r(3);
    
        a_value(index+1) = q(1);
        b_value(index+1) = q(2);
        c_value(index+1) = q(3);
        d_value(index+1) = q(4);
    
        [Heading, pitch, roll] = quat2angle(q,'ZYX');
    
        Euler_roll_value(index + 1) = roll;
        Euler_pitch_value(index + 1) = pitch;
        Euler_heading_value(index + 1) = Heading;
    
        ve_value(index + 1 ) = v(1);
        vn_value(index + 1 ) = v(2);
        vu_value(index + 1 ) = v(3);
    
        lon_value(index+1) = lon*R2D;
        lat_value(index+1) = lat*R2D;
    end

    %--> Calculate new meridian and normal radius of curvatures
    Rn_value(index+1)  = earth_a/sqrt(1-earth_e2*sin(lat_value(index+1)*D2R)*sin(lat_value(index+1)*D2R));
    Rm_value(index+1)  = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat_value(index+1)*D2R)*sin(lat_value(index+1)*D2R))^(1.5));
    
    %--> Advance biases (no change between filter updates)
    gyro_bias_x_value(index+1) = gyro_bias_x_value(index);
    gyro_bias_y_value(index+1) = gyro_bias_y_value(index);
    gyro_bias_z_value(index+1) = gyro_bias_z_value(index);
    acc_bias_x_value(index+1) = acc_bias_x_value(index);
    acc_bias_y_value(index+1) = acc_bias_y_value(index);
    acc_bias_z_value(index+1) = acc_bias_z_value(index);
    
    %--> Advance scale factors (no change between filter updates)
    gyro_sf_x_value(index+1) = gyro_sf_x_value(index);
    gyro_sf_y_value(index+1) = gyro_sf_y_value(index);
    gyro_sf_z_value(index+1) = gyro_sf_z_value(index);
    acc_sf_x_value(index+1) = acc_sf_x_value(index);
    acc_sf_y_value(index+1) = acc_sf_y_value(index);
    acc_sf_z_value(index+1) = acc_sf_z_value(index);
    
    %--> Kalman Filter model matrices calculation
    F_value = F_fun(Rm_value(index),Rn_value(index),raw_acc_x(index),raw_acc_y(index),raw_acc_z(index),...
        acc_sf_x_value(index),acc_sf_y_value(index),acc_sf_z_value(index),...
        acc_bias_x_value(index),acc_bias_y_value(index),acc_bias_z_value(index),...
        acc_rw_stdv_x_value(index),acc_rw_stdv_y_value(index),acc_rw_stdv_z_value(index),...
        acc_sf_x_time_cnst_value(index),acc_sf_y_time_cnst_value(index),acc_sf_z_time_cnst_value(index),...
        acc_bias_x_time_cnst_value(index),acc_bias_y_time_cnst_value(index),acc_bias_z_time_cnst_value(index),...
        alt_value(index),b_value(index),c_value(index),d_value(index),...
        raw_gyro_x(index),raw_gyro_y(index),raw_gyro_z(index),...
        gyro_sf_x_value(index),gyro_sf_y_value(index),gyro_sf_z_value(index),...
        gyro_bias_x_value(index),gyro_bias_y_value(index),gyro_bias_z_value(index),...
        gyro_rw_stdv_x_value(index),gyro_rw_stdv_y_value(index),gyro_rw_stdv_z_value(index),...
        gyro_sf_x_time_cnst_value(index),gyro_sf_y_time_cnst_value(index),gyro_sf_z_time_cnst_value(index),...
        gyro_bias_x_time_cnst_value(index),gyro_bias_y_time_cnst_value(index),gyro_bias_z_time_cnst_value(index),...
        lat_value(index)*R2D,ve_value(index),vn_value(index),vu_value(index),we_value,wg_noise_value(index));
    
    if (~isreal(F_value))
        disp('imaginary transition matrix');return;
    end
    
    Phi = eye(size(F_value)) + F_value*sampling_period_sec;
    
    G_value = G_fun(acc_rw_stdv_x_value(index),acc_rw_stdv_y_value(index),acc_rw_stdv_z_value(index),...
        acc_sf_x_time_cnst_value(index),acc_sf_y_time_cnst_value(index),acc_sf_z_time_cnst_value(index),...
        acc_bias_x_time_cnst_value(index),acc_bias_z_time_cnst_value(index),acc_bias_y_time_cnst_value(index),...
        acc_sf_gauss_markov_stdv_x_value(index),acc_sf_gauss_markov_stdv_y_value(index),acc_sf_gauss_markov_stdv_z_value(index),...
        acc_bias_gauss_markov_stdv_x_value(index),acc_bias_gauss_markov_stdv_y_value(index),acc_bias_gauss_markov_stdv_z_value(index),....
        alt_rw_stdv_value(index),...
        b_value(index), c_value(index),d_value(index),...
        gyro_rw_stdv_x_value(index),gyro_rw_stdv_y_value(index),gyro_rw_stdv_z_value(index),...
        gyro_sf_x_time_cnst_value(index),gyro_sf_y_time_cnst_value(index),gyro_sf_z_time_cnst_value(index),...
        gyro_bias_x_time_cnst_value(index),gyro_bias_y_time_cnst_value(index),gyro_bias_z_time_cnst_value(index),...
        gyro_sf_gauss_markov_stdv_x_value(index),gyro_sf_gauss_markov_stdv_y_value(index),gyro_sf_gauss_markov_stdv_z_value(index),...
        gyro_bias_gauss_markov_stdv_x_value(index),gyro_bias_gauss_markov_stdv_y_value(index),gyro_bias_gauss_markov_stdv_z_value(index));
    
    if (~isreal(G_value))
        disp('imaginary noise shaping matrix');return;
    end
    
    Qd = sampling_period_sec^2*(G_value*G_value');
    
    %--> EKF prediction
    P = Phi*P*Phi' + Qd;
    
    if (~isreal(P))
        disp('imaginary noise covariance matrix');return;
    end
    
    %--> apply updates from noisy obervations
    if (mod(index,sampling_freq) == 0 && forward_kinematics_only == 0)
        %--> Initialize error vector and matrices
        clear z H R;
        state_correction_vector = zeros(length(P),1);
        
        %--> set the innovation sequence       
        z(1) = ems_data.east_noisy(index)        - EP_value(index+1);
        z(2) = ems_data.north_noisy(index)       - NP_value(index+1);
        z(3) = ems_data.h_noisy(index)          - alt_value(index+1);
        z(4) = ems_data.vel_N_noisy(index,1)        - ve_value(index+1);
        z(5) = ems_data.vel_N_noisy(index,2)       - vn_value(index+1);
        z(6) = ems_data.vel_N_noisy(index,3)          - vu_value(index+1);
        
        H              = zeros(6,21);
        H(1,1)         = 1;
        H(2,2)         = 1;
        H(3,3)         = 1;
        H(4,4)         = 1;
        H(5,5)         = 1;
        H(6,6)         = 1;
        
        R              = diag([10 10 10 .5 .5 .5].^2);
        
        K = P*H'/(H*P*H'+R);
        state_correction_vector = state_correction_vector + K*z';
        P = P - K*H*P;
               
        if (~isreal(P))
            disp('imaginary updated error covariance matrix ');return;
        end
        
        correct_states;
    end
    
    %--> Normalize P
    P = (P+P')/2;
    
    if (~isreal(P))
        disp('imaginary error covariance matrix ');return;
    end
    
    gyro_rw_stdv_x_value(index+1) = gyro_rw_stdv_x_value(index);
    gyro_rw_stdv_y_value(index+1) = gyro_rw_stdv_y_value(index);
    gyro_rw_stdv_z_value(index+1) = gyro_rw_stdv_z_value(index);   
    gyro_bias_gauss_markov_stdv_x_value (index+1) = gyro_bias_gauss_markov_stdv_x_value (index);
    gyro_bias_gauss_markov_stdv_y_value (index+1) = gyro_bias_gauss_markov_stdv_y_value (index);
    gyro_bias_gauss_markov_stdv_z_value (index+1) = gyro_bias_gauss_markov_stdv_z_value (index);
    gyro_bias_x_time_cnst_value(index+1) = gyro_bias_x_time_cnst_value(index);
    gyro_bias_y_time_cnst_value(index+1) = gyro_bias_y_time_cnst_value(index);
    gyro_bias_z_time_cnst_value(index+1) = gyro_bias_z_time_cnst_value(index);
    gyro_sf_gauss_markov_stdv_x_value (index+1) = gyro_sf_gauss_markov_stdv_x_value (index);
    gyro_sf_gauss_markov_stdv_y_value (index+1) = gyro_sf_gauss_markov_stdv_y_value (index);
    gyro_sf_gauss_markov_stdv_z_value (index+1) = gyro_sf_gauss_markov_stdv_z_value (index);
    gyro_sf_x_time_cnst_value(index+1) = gyro_sf_x_time_cnst_value(index);
    gyro_sf_y_time_cnst_value(index+1) = gyro_sf_y_time_cnst_value(index);
    gyro_sf_z_time_cnst_value(index+1) = gyro_sf_z_time_cnst_value(index);
    
    acc_rw_stdv_x_value(index+1) = acc_rw_stdv_x_value(index);
    acc_rw_stdv_y_value(index+1) = acc_rw_stdv_y_value(index);
    acc_rw_stdv_z_value(index+1) = acc_rw_stdv_z_value(index);
    acc_bias_gauss_markov_stdv_x_value (index+1) = acc_bias_gauss_markov_stdv_x_value (index);
    acc_bias_gauss_markov_stdv_y_value (index+1) = acc_bias_gauss_markov_stdv_y_value (index);
    acc_bias_gauss_markov_stdv_z_value (index+1) = acc_bias_gauss_markov_stdv_z_value (index);
    acc_bias_x_time_cnst_value(index+1) = acc_bias_x_time_cnst_value(index);
    acc_bias_y_time_cnst_value(index+1) = acc_bias_y_time_cnst_value(index);
    acc_bias_z_time_cnst_value(index+1) = acc_bias_z_time_cnst_value(index);
    acc_sf_gauss_markov_stdv_x_value (index+1) = acc_sf_gauss_markov_stdv_x_value (index);
    acc_sf_gauss_markov_stdv_y_value (index+1) = acc_sf_gauss_markov_stdv_y_value (index);
    acc_sf_gauss_markov_stdv_z_value (index+1) = acc_sf_gauss_markov_stdv_z_value (index);
    acc_sf_x_time_cnst_value(index+1) = acc_sf_x_time_cnst_value(index);
    acc_sf_y_time_cnst_value(index+1) = acc_sf_y_time_cnst_value(index);
    acc_sf_z_time_cnst_value(index+1) = acc_sf_z_time_cnst_value(index);
        
    alt_rw_stdv_value(index + 1)   = alt_rw_stdv_value(index);
    
    wg_noise_value(index+1) = 0.0;

    g_value(index + 1) = 9.8;
    
    % --> Save covariance values for display
    east_pos_error_covariance(index + 1) = P(1,1);
    north_pos_error_covariance(index + 1) = P(2,2);
    alt_error_covariance(index + 1) = P(3,3);
    ve_error_covariance(index + 1) = P(4,4);
    vn_error_covariance(index + 1) = P(5,5);
    vu_error_covariance(index + 1) = P(6,6);
    b_error_covariance(index + 1) = P(7,7);
    c_error_covariance(index + 1) = P(8,8);
    d_error_covariance(index + 1) = P(9,9);
    gyro_bias_x_error_covariance(index + 1) = P(10,10);
    gyro_bias_y_error_covariance(index + 1) = P(11,11);
    gyro_bias_z_error_covariance(index + 1) = P(12,12);
    acc_bias_x_error_covariance(index + 1) = P(13,13);
    acc_bias_y_error_covariance(index + 1) = P(14,14);
    acc_bias_z_error_covariance(index + 1) = P(15,15);
    gyro_sf_x_error_covariance(index + 1) = P(16,16);
    gyro_sf_y_error_covariance(index + 1) = P(17,17);
    gyro_sf_z_error_covariance(index + 1) = P(18,18);
    acc_sf_x_error_covariance(index + 1) = P(19,19);
    acc_sf_y_error_covariance(index + 1) = P(20,20);
    acc_sf_z_error_covariance(index + 1) = P(21,21);

    fprintf('processing epoch %d/%d\n',index,num_of_samples);
end

q_ref = angle2quat(ems_data.heading,ems_data.pitch,ems_data.roll,'ZYX');

%--> Print error values
v_east_ref_vector       = ems_data.vel_N(:,1)';
v_north_ref_vector      = ems_data.vel_N(:,2)';
v_up_ref_vector         = ems_data.vel_N(:,3)';
p_east_ref_vector       = ems_data.east;
p_north_ref_vector      = ems_data.north;
alt_ref_vector          = ems_data.h;
roll_ref_vector         = ems_data.roll;
pitch_ref_vector        = ems_data.pitch;
heading_ref_vector      = ems_data.heading;
fprintf('V east error(m/s) = %.10f\n', sqrt(mean((v_east_ref_vector'-ve_value).^2)));
fprintf('V north error(m/s) = %.10f\n', sqrt(mean((v_north_ref_vector'-vn_value).^2)));
fprintf('V up error(m/s) = %.10f\n', sqrt(mean((v_up_ref_vector'-vu_value).^2)));
fprintf('East error(m) = %.10f\n', sqrt(mean((p_east_ref_vector-EP_value).^2)));
fprintf('North error(m) = %.10f\n', sqrt(mean((p_north_ref_vector-NP_value).^2)));
fprintf('Alt error(m) = %.10f\n', sqrt(mean((alt_ref_vector-alt_value).^2)));
fprintf('Roll error(deg) = %.10f\n', sqrt(mean((roll_ref_vector-Euler_roll_value).^2))*R2D);
fprintf('Pitch error(deg) = %.10f\n', sqrt(mean((pitch_ref_vector-Euler_pitch_value).^2))*R2D);
fprintf('Heading error(deg) = %.10f\n', sqrt(mean((heading_ref_vector-Euler_heading_value).^2))*R2D);

%--> Display results figures
display_results