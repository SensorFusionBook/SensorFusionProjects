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

%--> Load EKF parameters
load('EKF_parameters');

%--> Load the dataset
load('ems_data_TC');

%--> Call symbolic mechanization equation definition script
ems_symbolic_engine_TC;

%--> Enable/Disable animation of results
enable_animation = 1;

%--> Converting symbolic functions into real-valued functions
C_LB_from_Euler_fun         = matlabFunction(C_LB_from_Euler);
w_L_IL_fun                  = matlabFunction(w_L_IL);
a_dot_fun                   = matlabFunction(a_dot);
b_dot_fun                   = matlabFunction(b_dot);
c_dot_fun                   = matlabFunction(c_dot);
d_dot_fun                   = matlabFunction(d_dot);
C_LB_from_quat_fun          = matlabFunction(C_LB_from_quat);
C_EN_fun                    = matlabFunction(C_EN);
C_EL_fun                    = matlabFunction(C_EL);
w_N_EN_fun                  = matlabFunction(w_N_EN);
w_N_IE_fun                  = matlabFunction(w_N_IE);
V_N_dot_fun                 = matlabFunction(V_N_dot);
pos_quat_dot_fun_EN         = matlabFunction(pos_quat_dot_EN);
pos_quat_dot_fun_EL         = matlabFunction(pos_quat_dot_EL);
C_EN_from_pos_quat_fun      = matlabFunction(C_EN_from_pos_quat);
C_EL_from_pos_quat_fun      = matlabFunction(C_EL_from_pos_quat);
pos_quat_from_lat_lon_fun   = matlabFunction(pos_quat_from_lat_lon);
w_L_IE_fun                  = matlabFunction(w_L_IE);
gravity_fun                 = matlabFunction(g);
F_fun                       = matlabFunction(F);
G_fun                       = matlabFunction(G);
H_row_range_fun             = matlabFunction(H_row_range);

%--> Earth ellipsoid shape parameters
earth_a           = 6378137;
earth_f           = 1/298.257223563;
earth_b           = earth_a*(1-earth_f);
earth_e2          = 1-(earth_b^2)/(earth_a^2);
we_value          = 2*pi/(24*60*60);


%--> Initialization
num_of_samples = length(ems_data.time);
sampling_period_sec = mean(diff(ems_data.time));
sampling_freq = round(1/sampling_period_sec);

%--> Initialize vectors sizes
time_vector_sec                     = ems_data.time;
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
a_pos_value                             = zeros(num_of_samples,1);
b_pos_value                             = zeros(num_of_samples,1);
c_pos_value                             = zeros(num_of_samples,1);
d_pos_value                             = zeros(num_of_samples,1);
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

a_pos_rw_stdv_value                     = zeros(num_of_samples,1);
b_pos_rw_stdv_value                     = zeros(num_of_samples,1);
c_pos_rw_stdv_value                     = zeros(num_of_samples,1);
d_pos_rw_stdv_value                     = zeros(num_of_samples,1);
alt_rw_stdv_value                       = zeros(num_of_samples,1);

receiver_clk_bias_value                 = zeros(num_of_samples,1);
receiver_clk_bias_stdv_value            = zeros(num_of_samples,1);
receiver_clk_drift_value                = zeros(num_of_samples,1);
receiver_clk_drift_stdv_value           = zeros(num_of_samples,1);

wg_noise_value                          = zeros(num_of_samples,1);

%--> Initialize the state
lat0 = ems_data.lat(1);
lon0 = ems_data.lon(1);
alt0 = ems_data.h(1);

Euler_roll_value(1)     = ems_data.roll(1);
Euler_pitch_value(1)    = ems_data.pitch(1);
Euler_heading_value(1)  = ems_data.heading(1);
attitude_quat_value = angle2quat(Euler_heading_value(1),Euler_pitch_value(1),Euler_roll_value(1),'ZYX');
C_BL_value = angle2dcm(Euler_heading_value(1),Euler_pitch_value(1),Euler_roll_value(1),'ZYX');
C_LB_value = C_BL_value';
a_value(1) = attitude_quat_value(1);
b_value(1) = attitude_quat_value(2);
c_value(1) = attitude_quat_value(3);
d_value(1) = attitude_quat_value(4);
lat_value(1)  = ems_data.lat(1)*R2D;
lon_value(1)  = ems_data.lon(1)*R2D;
alt_value(1)  = ems_data.h(1);
Rn_value(1)   = earth_a/sqrt(1-earth_e2*sin(lat_value(1)*D2R)*sin(lat_value(1)*D2R));
Rm_value(1)   = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat_value(1)*D2R)*sin(lat_value(1)*D2R))^(1.5));
EP_value(1) = ems_data.east(1);
NP_value(1) = ems_data.north(1);
g_value(1) = gravity_fun(Rm_value(1),Rn_value(1),alt_value(1),lat_value(1));
C_EN_value = C_EN_fun(lat_value(1)*D2R , lon_value(1)*D2R);
pos_quat_vector = dcm2quat (C_EN_value');
a_pos_value(1) = pos_quat_vector(1)/sqrt(sum(pos_quat_vector.^2));
b_pos_value(1) = pos_quat_vector(2)/sqrt(sum(pos_quat_vector.^2));
c_pos_value(1) = pos_quat_vector(3)/sqrt(sum(pos_quat_vector.^2));
d_pos_value(1) = pos_quat_vector(4)/sqrt(sum(pos_quat_vector.^2));
ve_value(1) = ems_data.vel_N(1,1);
vn_value(1) = ems_data.vel_N(1,2);
vu_value(1) = ems_data.vel_N(1,3);

%--> Initialize biases
gyro_bias_x_value(1) = 0.0;
gyro_bias_y_value(1) = 0.0;
gyro_bias_z_value(1) = 0.0;
acc_bias_x_value(1)  = 0.0;
acc_bias_y_value(1)  = 0.0;
acc_bias_z_value(1)  = 0.0;

%--> Initialize scale factors
gyro_sf_x_value(1) = 0.0;
gyro_sf_y_value(1) = 0.0;
gyro_sf_z_value(1) = 0.0;
acc_sf_x_value(1)  = 0.0;
acc_sf_y_value(1)  = 0.0;
acc_sf_z_value(1)  = 0.0;

%--> Noise is to set to zero in INS prediction
wg_noise_value(1) = 0.0;

%--> Position system noise in Q matrix
a_pos_rw_stdv_value(1)  = 0.0;
b_pos_rw_stdv_value(1)  = 0.0;
c_pos_rw_stdv_value(1)  = 0.0;
d_pos_rw_stdv_value(1)  = 0.0;
alt_rw_stdv_value(1)    = 0.0;

%--> Accelerometer System noise and Gauss-Markov model parameters
acc_rw_stdv_x_value(1) = parameters(1);
acc_rw_stdv_y_value(1) = parameters(2);
acc_rw_stdv_z_value(1) = parameters(3);
acc_bias_gauss_markov_stdv_x_value (1) = parameters(4);
acc_bias_gauss_markov_stdv_y_value (1) = parameters(5);
acc_bias_gauss_markov_stdv_z_value (1) = parameters(6);
acc_bias_x_time_cnst_value(1) = parameters(7);
acc_bias_y_time_cnst_value(1) = parameters(8);
acc_bias_z_time_cnst_value(1) = parameters(9);
acc_sf_gauss_markov_stdv_x_value (1) = parameters(10);
acc_sf_gauss_markov_stdv_y_value (1) = parameters(11);
acc_sf_gauss_markov_stdv_z_value (1) = parameters(12);
acc_sf_x_time_cnst_value(1) = parameters(13);
acc_sf_y_time_cnst_value(1) = parameters(14);
acc_sf_z_time_cnst_value(1) = parameters(15);

%--> Gyroscope System noise and Gauss-Markov model parameters
gyro_rw_stdv_x_value(1) = parameters(16);
gyro_rw_stdv_y_value(1) = parameters(17);
gyro_rw_stdv_z_value(1) = parameters(18);
gyro_bias_gauss_markov_stdv_x_value (1) = parameters(19);
gyro_bias_gauss_markov_stdv_y_value (1) = parameters(20);
gyro_bias_gauss_markov_stdv_z_value (1) = parameters(21);
gyro_bias_x_time_cnst_value(1) = parameters(22);
gyro_bias_y_time_cnst_value(1) = parameters(23);
gyro_bias_z_time_cnst_value(1) = parameters(24);
gyro_sf_gauss_markov_stdv_x_value (1) = parameters(25);
gyro_sf_gauss_markov_stdv_y_value (1) = parameters(26);
gyro_sf_gauss_markov_stdv_z_value (1) = parameters(27);
gyro_sf_x_time_cnst_value(1) = parameters(28);
gyro_sf_y_time_cnst_value(1) = parameters(29);
gyro_sf_z_time_cnst_value(1) = parameters(30);

%--> Receiver clock errors parameters
receiver_clk_bias_stdv_value(1) = parameters(31);
receiver_clk_drift_stdv_value(1) = parameters(32);

%--> Initialize state error covariances
east_pos_error_covariance(1) = parameters(33);
north_pos_error_covariance(1) = parameters(34);
alt_error_covariance(1) = parameters(35);
ve_error_covariance(1) = parameters(36);
vn_error_covariance(1) = parameters(37);
vu_error_covariance(1) = parameters(38);
b_error_covariance(1) = parameters(39);
c_error_covariance(1) = parameters(40);
d_error_covariance(1) = parameters(41);
gyro_bias_x_error_covariance(1) = parameters(42);
gyro_bias_y_error_covariance(1) = parameters(43);
gyro_bias_z_error_covariance(1) = parameters(44);
acc_bias_x_error_covariance(1) = parameters(45);
acc_bias_y_error_covariance(1) = parameters(46);
acc_bias_z_error_covariance(1) = parameters(47);
gyro_sf_x_error_covariance(1) = parameters(48);
gyro_sf_y_error_covariance(1) = parameters(49);
gyro_sf_z_error_covariance(1) =parameters(50);
acc_sf_x_error_covariance(1) = parameters(51);
acc_sf_y_error_covariance(1) = parameters(52);
acc_sf_z_error_covariance(1) = parameters(53);
receiver_clk_bias_error_covariance(1) =parameters(54);
receiver_clk_drift_error_covariance(1) =parameters(55);

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
    acc_sf_z_error_covariance(1);
    receiver_clk_bias_error_covariance(1);
    receiver_clk_drift_error_covariance(1)
    ]);

%--> Draw a 3D Aircraft for animation
X = 60;Y = 15;Z = 5;
origin = [X/2 Y/2 Z/2];
initial_vert = ...
    [X Y 0;             %(1)
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

CubePoints = [initial_vert(:,1),initial_vert(:,2),initial_vert(:,3)];
[ CubeXData , CubeYData , CubeZData ] = get_cube_axis_data(initial_vert);
faces = [1 2 3 4; 4 3 5 6; 6 7 8 5; 1 2 8 7; 6 7 1 4; 2 3 5 8;1 9 10 4;4 10 6 6;6 10 9 7;1 9 7 7];

%--> Main processing loop
for index = 1:length(ems_data.time)-1
      
    %--> Read IMU data
    raw_acc_x(index) = ems_data.accel(index,1);
    raw_acc_y(index) = ems_data.accel(index,2);
    raw_acc_z(index) = ems_data.accel(index,3);

    raw_gyro_x(index) = ems_data.gyro(index,1);
    raw_gyro_y(index) = ems_data.gyro(index,2);
    raw_gyro_z(index) = ems_data.gyro(index,3);      

    %--> Apply forward kinematics

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
    
    %--> Calculate new earth parameters and east, north position components
    Rn_value(index+1)  = earth_a/sqrt(1-earth_e2*sin(lat_value(index+1)*D2R)*sin(lat_value(index+1)*D2R));
    Rm_value(index+1)  = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat_value(index+1)*D2R)*sin(lat_value(index+1)*D2R))^(1.5));
    
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
    
    %--> Advance receiver clock bias and drift
    receiver_clk_bias_value(index+1) = receiver_clk_bias_value(index);
    receiver_clk_drift_value(index+1) = receiver_clk_drift_value(index);
    
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
                    gyro_bias_gauss_markov_stdv_x_value(index),gyro_bias_gauss_markov_stdv_y_value(index),gyro_bias_gauss_markov_stdv_z_value(index),...
                    receiver_clk_bias_stdv_value(index), receiver_clk_drift_stdv_value(index));

    if (~isreal(G_value))
        disp('imaginary noise shaping matrix');return;
    end
    
    Qd = sampling_period_sec^2*(G_value*G_value');
    
    %--> Advance state error covariance matrix
    P = Phi*P*Phi' + Qd;
    
    if (~isreal(P))
        disp('imaginary noise covariance matrix');return;
    end
    
    %--> Apply updates from obervations
    if (mod(index,sampling_freq) == 0 && ~(ems_data.time(index) > 140 && ems_data.time(index) < 160))
        %--> Initialize error vector and matrices
        clear z H R;
        
        no_of_sats = ems_data.no_of_sat{((index)/sampling_freq)+1};        
        
        if (ems_data.time(index) > 140 && ems_data.time(index) < 160)
            no_of_sats = 1;
            ems_data.no_of_sat{((index)/sampling_freq)+1} = 1;
            sat_Px = ems_data.sat_Px{((index)/sampling_freq)+1};
            sat_Py = ems_data.sat_Py{((index)/sampling_freq)+1};
            sat_Pz = ems_data.sat_Pz{((index)/sampling_freq)+1};
            sat_range = ems_data.sat_range{((index)/sampling_freq)+1};
            ems_data.sat_Px{((index)/sampling_freq)+1} = sat_Px(1:no_of_sats);
            ems_data.sat_Py{((index)/sampling_freq)+1} = sat_Py(1:no_of_sats);
            ems_data.sat_Pz{((index)/sampling_freq)+1} = sat_Pz(1:no_of_sats);
            ems_data.sat_range{((index)/sampling_freq)+1} = sat_range(1:no_of_sats);
            ems_data.no_of_sat{((index)/sampling_freq)+1} = no_of_sats;
        end

        %--> Convert reciever position from Geodetic frame to ECEF frame
        x = (Rn_value(index+1) + alt_value(index+1))*cos(lat_value(index+1)*D2R)*cos(lon_value(index+1)*D2R);
        y = (Rn_value(index+1) + alt_value(index+1))*cos(lat_value(index+1)*D2R)*sin(lon_value(index+1)*D2R);
        z = ((Rn_value(index+1)*(1-earth_e2))+alt_value(index+1))*sin(lat_value(index+1)*D2R);        
        
        %--> Set the innovation sequence
        deltaPx = (x.*ones(no_of_sats,1)) - (ems_data.sat_Px{((index)/sampling_freq)+1});
        deltaPy = (y.*ones(no_of_sats,1)) - (ems_data.sat_Py{((index)/sampling_freq)+1});
        deltaPz = (z.*ones(no_of_sats,1)) - (ems_data.sat_Pz{((index)/sampling_freq)+1});

        range_from_INS = sqrt(deltaPx.^2+deltaPy.^2+deltaPz.^2);      
        range_from_GNSS = cell2mat(ems_data.sat_range(((index)/sampling_freq)+1)) + (ems_data.noiseInfo.clk_bias - receiver_clk_bias_value(index+1)).*ones(no_of_sats,1);
        
        z = range_from_GNSS - range_from_INS;
        
        %--> Set the measurment matrix jacobian
        H = zeros(no_of_sats, 23);
        
        for m=1:no_of_sats
            H(m,:) = H_row_range_fun(Rm_value(index+1),Rn_value(index+1),alt_value(index+1),...
                                    (lon_value(index+1)*D2R - ems_data.lon(1))*(Rn_value(index+1)+alt_value(index+1))*cos(lat_value(index+1)*D2R)+ems_data.east(1),...
                                    ems_data.east(1),ems_data.lat(1),ems_data.lon(1),(lat_value(index+1)*D2R - ems_data.lat(1))*(Rm_value(index+1)+alt_value(index+1)),...
                                    ems_data.north(1),ems_data.sat_Px{(index/sampling_freq)+1}(m),ems_data.sat_Py{(index/sampling_freq)+1}(m),ems_data.sat_Pz{(index/sampling_freq)+1}(m));
        end
        
        %--> Adjust measurment noise matrix
        R = diag(ones(1*no_of_sats,1));   
        
        range_error_std = parameters(56);
       
        R(1:no_of_sats,1:no_of_sats) = R(1:no_of_sats,1:no_of_sats).*(range_error_std.^2);
        
        
        if (~isreal(P))
            disp('imaginary updated error covariance matrix ');return;
        end
        
        %--> EKF update
        K = (P*H')/(H*P*H'+R);
        state_correction_vector = K*z;
        P = P - K*H*P;
        
        %--> Normalize P
        P = (P+P')/2;

        if (~isreal(P))
            disp('imaginary error covariance matrix ');return;
        end
        
        %--> Correct states
        correct_states_TC;
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
    
    a_pos_rw_stdv_value(index + 1) = a_pos_rw_stdv_value(index);
    b_pos_rw_stdv_value(index + 1) = b_pos_rw_stdv_value(index);
    c_pos_rw_stdv_value(index + 1) = c_pos_rw_stdv_value(index);
    d_pos_rw_stdv_value(index + 1) = d_pos_rw_stdv_value(index);
    alt_rw_stdv_value(index + 1)   = alt_rw_stdv_value(index);
    
    wg_noise_value(index+1) = 0.0;
    
    g_value(index + 1) = gravity_fun(Rm_value(index),Rn_value(index),alt_value(index),lat_value(index));
    
    %--> Disply results in 3D
    C_LB_value_plus_90 = C_LB_from_Euler_fun(Euler_roll_value(index),...
        -Euler_pitch_value(index),...
        pi/2 - Euler_heading_value(index));
    
    updated_vert = C_LB_value_plus_90*initial_vert';
    
    updated_vert = updated_vert';
    
    for p = 1:length(updated_vert)
        updated_vert(p,:) = updated_vert(p,:) + [EP_value(index), NP_value(index), alt_value(index)];
    end
    
    [ CubeXData , CubeYData , CubeZData ] = get_cube_axis_data(updated_vert);
    CubePoints = [updated_vert(:,1),updated_vert(:,2),updated_vert(:,3)];
    
    %--> Create first figure after the 5th IMU sample
    if(index == 5)
        figure;
        hold on;
        plot3(ems_data.east,ems_data.north,ems_data.h, 'LineWidth', 1, 'color', [0.93 .69 .13]);grid on;xlabel('east');ylabel('north');zlabel('alt');
        view(-1,50);title('3D Trajectory of Ground Truth, Noisy Updates, and EKF Solution');
        h_noisy_updates = plot3(ems_data.east_noisy(1:20:index),ems_data.north_noisy(1:20:index),ems_data.h_noisy(1:20:index), '-*','MarkerSize', 5,'LineWidth', 0.5, 'color','k');grid on;xlabel('east');ylabel('north');zlabel('alt');
        h_ekf_position  = plot3(EP_value(1:index), NP_value(1:index), alt_value(1:index), 'color', 'r','LineWidth', 2);hold on;grid on;
        set(h_ekf_position,'YDataSource','NP_value(1:index)');
        set(h_ekf_position,'XDataSource','EP_value(1:index)');
        set(h_ekf_position,'ZDataSource','alt_value(1:index)');
        set(h_noisy_updates,'YDataSource','ems_data.north_noisy(1:20:index)');
        set(h_noisy_updates,'XDataSource','ems_data.east_noisy(1:20:index)');
        set(h_noisy_updates,'ZDataSource','ems_data.h_noisy(1:20:index)');
        h_vehicle = patch('Faces',faces,'Vertices',CubePoints,'FaceVertexCData',hsv(10),'FaceColor','flat');
        set(h_vehicle,'XData',CubeXData,'YData',CubeYData,'ZData',CubeZData);
        set(gca,'FontSize',12);
        xlabel('East (m)');ylabel('North (m)');zlabel('Altitude(m)');
        axis([min(ems_data.east) max(ems_data.east) min(ems_data.north) max(ems_data.north) -100 300]);
        legend({'Ground Truth','Noisy GPS','EKF Solution'}, 'FontSize', 12); 
    else
        if enable_animation == 1
            if (mod(index,sampling_freq) == 0)
                refreshdata(h_ekf_position, 'caller');
                refreshdata(h_noisy_updates, 'caller');
                set(h_vehicle,'XData',CubeXData,'YData',CubeYData,'ZData',CubeZData);
                pause(0.05);
            end
        end
    end

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
    receiver_clk_bias_error_covariance(index + 1) = P(22,22); 
    receiver_clk_drift_error_covariance(index + 1) = P(23,23); 

    fprintf('processing epoch %d/%d\n',index,num_of_samples);
end

%--> Refresh figures
refreshdata(h_ekf_position, 'caller');
refreshdata(h_noisy_updates, 'caller');
set(h_vehicle,'XData',CubeXData,'YData',CubeYData,'ZData',CubeZData);

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