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

%--> Symbols definitions
reset(symengine);
syms    gyro_x gyro_y gyro_z;                                   % Gyrsocope Measurments
syms    gyro_x_std gyro_y_std gyro_z_std;                       % Gyroscope noise std
syms    gyro_x_bias gyro_y_bias gyro_z_bias;                    % Gyroscope bias
syms    gyro_x_bias_std gyro_y_bias_std gyro_z_bias_std;        % Gyroscope bias noise std
syms    gyro_x_scale gyro_y_scale gyro_z_scale;                 % Gyrsocope scale factor
syms    gyro_x_scale_std gyro_y_scale_std gyro_z_scale_std;     % Gyrsocope scale factor std

syms    accel_x accel_y accel_z;                                % Accelerometer Measurments 
syms    accel_x_std accel_y_std accel_z_std;                    % Accelerometer noise std 
syms    accel_x_bias accel_y_bias accel_z_bias;                 % Accelerometer bias
syms    accel_x_bias_std accel_y_bias_std accel_z_bias_std;     % accelelerometer bias std
syms    accel_x_scale accel_y_scale accel_z_scale;              % Accelerometer scale factor
syms    accel_x_scale_std accel_y_scale_std accel_z_scale_std;  % Gyrsocope scale factor std

syms    IMU_calib_a IMU_calib_b IMU_calib_c IMU_calib_d;        % IMU misalignemnt correction quaternion q_BI

syms    pos_lat pos_lon pos_alt;                                % Position Geodetic frame
syms    pos_north pos_east pos_down;                            % Position in L frame
% syms    pos_b pos_c pos_d;                                      % position using quaternion
% syms    pos_x pos_y pos_z;                                      % Positioni n ECEF frame

syms    vn ve vd;                                               % Velocity in L frame
% syms    vx vy vz;                                               % velocity in ECEF frame

syms    a b c d;                                                % Attitude in Quaternion q_LB
syms    Euler_roll Euler_pitch Euler_heading;                   % Attitude in Euler

% syms    sat_x sat_y sat_z;                                      % Satellite position in ECEF
% syms    sat_vx sat_vy sat_vz;                                   % Satellite velocity in ECEF
% syms    line_of_sight;                                          % Satellite line of sight vector
% syms    sat_range;                                              % Satellite range
% syms    sat_rangeRate;                                          % Satellite range rate
% syms    receiver_clk_bias;                                      % Receiver clock bias
% syms    receiver_clk_drift;                                     % Receiver clock drift

syms    f_x f_y o_x o_y;                                        % camera intrinsics [f_x f_y o_x o_y] (all in pixel unit)
syms    cam_exts_a cam_exts_b cam_exts_c cam_exts_d;            % IMU to Camera relative rotation quaternion  q_BC
syms    cam_exts_x cam_exts_y cam_exts_z;                       % IMU to Camera relative translation vector P_B_BC  

syms    X_f Y_f Z_f;                                            % Feature's camera coordinate
syms    x_f y_f z_f;                                            % Feature's image coordinate
syms    u_f_j_i v_f_j_i;                                        % Feature's pixel coordinate
syms    f_0_alpha f_0_beta f_0_prho Z_f_0;                      % Feature's inverse depth parametrization

syms    pos_north_0 pos_east_0 pos_down_0;                      % Camera position in N frame at time 0
syms    pos_north_i pos_east_i pos_down_i;                      % Camera position in N frame at time i
syms    att_a_0 att_b_0 att_c_0 att_d_0;                        % Camera attitude in N frame at time 0
syms    att_a_i att_b_i att_c_i att_d_i;                        % Camera attitude in N frame at time i

syms    wg_noise;                                               % white Gaussian noise

syms    q1 q2 q3 q4

%--> Degrees <-> Radians
D2R = pi/180;
R2D = 1/D2R;

%--> biases time constant
beta = 0.0001;

%--> Transformation matrices
C_NL = [0, 1,  0;
        1, 0,  0;
        0, 0, -1];
C_LN = C_NL';


%--> The earth parameters
earth_a = 6378137;
earth_f = 1/298.257223563;
earth_b = earth_a*(1-earth_f);
earth_e2 = 1-(earth_b^2)/(earth_a^2);

RN = earth_a/sqrt(1-earth_e2*(sin(pos_lat)^2));
RM = (earth_a*(1-earth_e2))/((1-earth_e2*(sin(pos_lat)^2))^1.5);

g_o = 9.780318 * ( 1 + 5.3024e-03.*sin(pos_lat).^2 - 5.9e-06.*(sin(2*pos_lat)).^2 );
g = (g_o ./ (1 + (pos_alt ./ sqrt(RN*RM))).^2);

g_N = [0; 0; -g];
g_L = C_LN*g_N;

we = (2*pi)/(24*60*60);
w_L_IE = [we*cos(pos_lat); 0; -we*sin(pos_lat)];
w_N_IE = [0; we*cos(pos_lat); we*sin(pos_lat)];


%--> IMU measurment compensation
% IMU_calib_a = sqrt(1- IMU_calib_b^2 - IMU_calib_c^2 - IMU_calib_d^2);

IMU_R_BI = [(IMU_calib_a^2+IMU_calib_b^2-IMU_calib_c^2-IMU_calib_d^2), 2*(IMU_calib_b*IMU_calib_c-IMU_calib_a*IMU_calib_d),       2*(IMU_calib_b*IMU_calib_d+IMU_calib_a*IMU_calib_c);
            2*(IMU_calib_b*IMU_calib_c+IMU_calib_a*IMU_calib_d),      (IMU_calib_a^2-IMU_calib_b^2+IMU_calib_c^2-IMU_calib_d^2), 2*(IMU_calib_c*IMU_calib_d-IMU_calib_a*IMU_calib_b);
            2*(IMU_calib_b*IMU_calib_d-IMU_calib_a*IMU_calib_c),      2*(IMU_calib_c*IMU_calib_d+IMU_calib_a*IMU_calib_b),     (IMU_calib_a^2-IMU_calib_b^2-IMU_calib_c^2+IMU_calib_d^2)];
      
% gyro_meas = [gyro_x; gyro_y; gyro_z]+...
%                       [gyro_x_std; gyro_y_std; gyro_z_std].*wg_noise;
% 
% gyro_scale = [gyro_x_scale; gyro_y_scale; gyro_z_scale];
% 
% gyro_B = (IMU_R_BI) * (gyro_scale.*gyro_meas - [gyro_x_bias; gyro_y_bias; gyro_z_bias]);
% 
% Omega = [0,         -gyro_B(1), -gyro_B(2), -gyro_B(3);...
%          gyro_B(1),  0,          gyro_B(3), -gyro_B(2);...
%          gyro_B(2), -gyro_B(3),  0,          gyro_B(1);...
%          gyro_B(3),  gyro_B(2), -gyro_B(1),  0];
% 
% accel_meas = [accel_x; accel_y; accel_z] +...
%                         [accel_x_std; accel_y_std; accel_z_std].*wg_noise;
% 
% accel_scale = [accel_x_scale; accel_y_scale; accel_z_scale];       
%        
% accel_B = (IMU_R_BI) * (accel_scale.*accel_meas - [accel_x_bias; accel_y_bias; accel_z_bias]);

gyro_meas = [gyro_x; gyro_y; gyro_z];

gyro_scale = [gyro_x_scale; gyro_y_scale; gyro_z_scale];

gyro_B = (IMU_R_BI) * (gyro_scale.*gyro_meas - [gyro_x_bias; gyro_y_bias; gyro_z_bias])+...
                      [gyro_x_std; gyro_y_std; gyro_z_std].*wg_noise;

Omega = [0,         -gyro_B(1), -gyro_B(2), -gyro_B(3);...
         gyro_B(1),  0,          gyro_B(3), -gyro_B(2);...
         gyro_B(2), -gyro_B(3),  0,          gyro_B(1);...
         gyro_B(3),  gyro_B(2), -gyro_B(1),  0];

accel_meas = [accel_x; accel_y; accel_z];

accel_scale = [accel_x_scale; accel_y_scale; accel_z_scale];       
       
accel_B = (IMU_R_BI) * (accel_scale.*accel_meas - [accel_x_bias; accel_y_bias; accel_z_bias]) +...
                        [accel_x_std; accel_y_std; accel_z_std].*wg_noise;


%--> IMU biases equations
gyro_x_bias_dot = -beta*gyro_x_bias + sqrt(2*beta)*gyro_x_bias_std*wg_noise;
gyro_y_bias_dot = -beta*gyro_y_bias + sqrt(2*beta)*gyro_y_bias_std*wg_noise;
gyro_z_bias_dot = -beta*gyro_z_bias + sqrt(2*beta)*gyro_z_bias_std*wg_noise;

gyro_x_scale_dot = -beta*gyro_x_scale + sqrt(2*beta)*gyro_x_scale_std*wg_noise;
gyro_y_scale_dot = -beta*gyro_y_scale + sqrt(2*beta)*gyro_y_scale_std*wg_noise;
gyro_z_scale_dot = -beta*gyro_z_scale + sqrt(2*beta)*gyro_z_scale_std*wg_noise;

accel_x_bias_dot = -beta*accel_x_bias + sqrt(2*beta)*accel_x_bias_std*wg_noise;
accel_y_bias_dot = -beta*accel_y_bias + sqrt(2*beta)*accel_y_bias_std*wg_noise;
accel_z_bias_dot = -beta*accel_z_bias + sqrt(2*beta)*accel_z_bias_std*wg_noise;

accel_x_scale_dot = -beta*accel_x_scale + sqrt(2*beta)*accel_x_scale_std*wg_noise;
accel_y_scale_dot = -beta*accel_y_scale + sqrt(2*beta)*accel_y_scale_std*wg_noise;
accel_z_scale_dot = -beta*accel_z_scale + sqrt(2*beta)*accel_z_scale_std*wg_noise;

IMU_calib_a_dot = 0;
IMU_calib_b_dot = 0;
IMU_calib_c_dot = 0;
IMU_calib_d_dot = 0;

%--> Attitude equations
% a = sqrt(1- b^2 - c^2 - d^2);

att_q_vect = [a; b; c; d];

att_R_LB = [(a^2+b^2-c^2-d^2), 2*(b*c-a*d),       2*(b*d+a*c);
           2*(b*c+a*d),       (a^2-b^2+c^2-d^2), 2*(c*d-a*b);
           2*(b*d-a*c),       2*(c*d+a*b),     (a^2-b^2-c^2+d^2)];

cp = cos(Euler_pitch);
ch = cos(Euler_heading);
sh = sin(Euler_heading);
cr = cos(Euler_roll);
sr = sin(Euler_roll);
sp = sin(Euler_pitch);

C_LB_Euler = [ cp*ch, -cr*sh + sr*sp*ch,  sr*sh + cr*sp*ch;
               cp*sh,  cr*ch + sr*sp*sh, -sr*ch + cr*sp*sh;
              -sp,     sr*cp,             cr*cp];
               
w_L_EL = [ ve/(RN+pos_alt); -vn/(RM+pos_alt); (-ve*tan(pos_lat))/(RN+pos_alt)];
w_N_EN = [-vn/(RM+pos_alt);  ve/(RN+pos_alt); (ve*tan(pos_lat))/(RN+pos_alt)];

w_B_IL = att_R_LB' * (w_L_IE + w_L_EL);

w_B_IL_skew = [0,         -w_B_IL(1), -w_B_IL(2), -w_B_IL(3);...
               w_B_IL(1),  0,          w_B_IL(3), -w_B_IL(2);...
               w_B_IL(2), -w_B_IL(3),  0,          w_B_IL(1);...
               w_B_IL(3),  w_B_IL(2), -w_B_IL(1),  0];

att_qL_LB  = [a, -b, -c, -d;
              b,  a, -d,  c;
              c,  d,  a, -b;
              d, -c,  b,  a];
                
att_q_dot_LB = 0.5.*(att_qL_LB*[0; gyro_B] - w_B_IL_skew*att_q_vect);

a_dot = att_q_dot_LB(1);
b_dot = att_q_dot_LB(2);
c_dot = att_q_dot_LB(3);
d_dot = att_q_dot_LB(4);


%--> Velocity equations
vel = [vn; ve; vd];
vu = -vd;
vel_N = [ve; vn; vu];

vel_dot = att_R_LB*accel_B + g_L - cross((2*w_L_IE + w_L_EL), vel);
vel_N_dot = C_NL * vel_dot;

vn_dot = vel_dot(1); 
ve_dot = vel_dot(2);
vd_dot = vel_dot(3);
vu_dot = -vel_dot(3);


%--> Position equations
pos_east_dot = ve;
pos_north_dot = vn;
pos_alt_dot = vu;
pos_down_dot = vd;

pos_lat_dot = vn/(RM+pos_alt);
pos_lon_dot = ve/((RN+pos_alt)*cos(pos_lat));


%--> GPS receiver clock bias and drift equations
% receiver_clk_bias_dot   = receiver_clk_drift + receiver_clk_bias_stdv*wg_noise;
% receiver_clk_drift_dot  = receiver_clk_drift_stdv*wg_noise;
% 
% %--> Geodetic and ENU position equations
% pos_lat = (pos_north - pos_north0)/(RM+pos_alt) + pos_lat0;
% pos_lon = (pos_east - pos_east0)/((RN+pos_alt)*cos(pos_lat)) + pos_lon0;
% 
% %--> Geodetic and ECEF position equations
% pos_x = (RN+pos_alt)*cos(pos_lat)*cos(pos_lon);
% pos_y = (RN+pos_alt)*cos(pos_lat)*sin(pos_lon);
% pos_z = (RN*(1-earth_e2)+pos_alt)*sin(pos_lat);
% 
% %--> Geodetic and ECEF velocity repos_lation equations
% vx = -(RN+pos_alt)*sin(pos_lat)*cos(pos_lon)*pos_lat_dot - (RN+pos_alt)*cos(pos_lat)*sin(pos_lon)*pos_lon_dot + cos(pos_lat)*cos(pos_lon)*pos_alt_dot;
% vy = -(RN+pos_alt)*sin(pos_lat)*sin(pos_lon)*pos_lat_dot + (RN+pos_alt)*cos(pos_lat)*cos(pos_lon)*pos_lon_dot + cos(pos_lat)*sin(pos_lon)*pos_alt_dot;
% vz = (RN*(1-earth_e2)+pos_alt)*cos(pos_lat)*pos_lat_dot + sin(pos_lat)*pos_alt_dot;
% 
% %--> Range and range rate equations
% sat_range = sqrt((pos_x - sat_x)^2 + (pos_y - sat_y)^2 + (pos_z - sat_z)^2) + receiver_clk_bias;
% line_of_sight = [(pos_x - sat_x), (pos_y - sat_y), (pos_z - sat_z)]./sat_range;
% sat_rangeRate = (line_of_sight*[(vx - sat_vx); (vy - sat_vy); (vz - sat_vz)]) + receiver_clk_drift;

%--> Camera intrinsics and extrinsics equations
f_x_dot = 0;
f_y_dot = 0;
o_x_dot = 0;
o_y_dot = 0;

cam_exts_a_dot = 0;
cam_exts_b_dot = 0;
cam_exts_c_dot = 0;
cam_exts_d_dot = 0;

cam_exts_x_dot = 0;
cam_exts_y_dot = 0;
cam_exts_z_dot = 0;


%--> Camera re-projection error equation                          
% cam_exts_a = sqrt(1- cam_exts_b^2 - cam_exts_c^2 - cam_exts_d^2); %q_BC

cam_exts_R_BC = [(cam_exts_a^2+cam_exts_b^2-cam_exts_c^2-cam_exts_d^2), 2*(cam_exts_b*cam_exts_c-cam_exts_a*cam_exts_d),       2*(cam_exts_b*cam_exts_d+cam_exts_a*cam_exts_c);
                2*(cam_exts_b*cam_exts_c+cam_exts_a*cam_exts_d),       (cam_exts_a^2-cam_exts_b^2+cam_exts_c^2-cam_exts_d^2), 2*(cam_exts_c*cam_exts_d-cam_exts_a*cam_exts_b);
                2*(cam_exts_b*cam_exts_d-cam_exts_a*cam_exts_c),       2*(cam_exts_c*cam_exts_d+cam_exts_a*cam_exts_b),     (cam_exts_a^2-cam_exts_b^2-cam_exts_c^2+cam_exts_d^2)];

% att_a_0 = sqrt(1- att_b_0^2 - att_c_0^2 - att_d_0^2); % q_LB0

att_R_LB0 = [(att_a_0^2+att_b_0^2-att_c_0^2-att_d_0^2), 2*(att_b_0*att_c_0-att_a_0*att_d_0),       2*(att_b_0*att_d_0+att_a_0*att_c_0);
             2*(att_b_0*att_c_0+att_a_0*att_d_0),       (att_a_0^2-att_b_0^2+att_c_0^2-att_d_0^2), 2*(att_c_0*att_d_0-att_a_0*att_b_0);
             2*(att_b_0*att_d_0-att_a_0*att_c_0),       2*(att_c_0*att_d_0+att_a_0*att_b_0),     (att_a_0^2-att_b_0^2-att_c_0^2+att_d_0^2)];
     
% att_a_i = sqrt(1- att_b_i^2 - att_c_i^2 - att_d_i^2); % q_LBi

att_R_LBi = [(att_a_i^2+att_b_i^2-att_c_i^2-att_d_i^2), 2*(att_b_i*att_c_i-att_a_i*att_d_i),       2*(att_b_i*att_d_i+att_a_i*att_c_i);
            2*(att_b_i*att_c_i+att_a_i*att_d_i),       (att_a_i^2-att_b_i^2+att_c_i^2-att_d_i^2), 2*(att_c_i*att_d_i-att_a_i*att_b_i);
            2*(att_b_i*att_d_i-att_a_i*att_c_i),       2*(att_c_i*att_d_i+att_a_i*att_b_i),     (att_a_i^2-att_b_i^2-att_c_i^2+att_d_i^2)];             

cam_pos_0 = [pos_north_0; pos_east_0; pos_down_0] + att_R_LB0*([cam_exts_x; cam_exts_y; cam_exts_z]); %P_LC0 = P_LB0 + R_LB0*P_B_BC

cam_pos_i = [pos_north_i; pos_east_i; pos_down_i] + att_R_LBi*([cam_exts_x; cam_exts_y; cam_exts_z]); %P_LCi = P_LBi + R_LBi*P_B_BC
 
cam_att_R_LC0 = att_R_LB0*cam_exts_R_BC; % R_LC0 = R_LB0*R_BC

cam_att_R_LCi = att_R_LBi*cam_exts_R_BC; % R_LCi = R_LBi*R_BC

R_CiC0 = cam_att_R_LCi' *cam_att_R_LC0;

P_Ci_CiC0 = cam_att_R_LCi' *(cam_pos_0 - cam_pos_i);

theta_C0_fj = [f_0_alpha; f_0_beta; f_0_prho];

P_Ci_Cifj = (Z_f_0).*((R_CiC0*[theta_C0_fj(1); theta_C0_fj(2); 1]) + (theta_C0_fj(3).*P_Ci_CiC0));

U_f_j_i = [o_x; o_y] + [f_x, 0; 0, f_y]*[P_Ci_Cifj(1)/P_Ci_Cifj(3); P_Ci_Cifj(2)/P_Ci_Cifj(3)];

r_j_i = [u_f_j_i; v_f_j_i] - U_f_j_i;


%--> Jacobian of Euler angles with respect to Quaternions (p.84 "UCGE Reports Development of a Real-Time Attitude System Using a Quaternion...")
r11 = 2.*(q2.*q3 + q1.*q4);
r12 = q1.^2 + q2.^2 - q3.^2 - q4.^2;
r21 = -2.*(q2.*q4 - q1.*q3);
r31 = 2.*(q3.*q4 + q1.*q2);
r32 = q1.^2 - q2.^2 - q3.^2 + q4.^2;
        
Euler_phi_q = atan2(r31, r32);
Euler_theta_q = real(asin(r21));
Euler_psi_q = atan2(r11, r12);

%--> states list
fixed_state_list = [pos_north; pos_east; pos_down;...
                    vn; ve; vd; a; b; c; d;... 
                    gyro_x_bias; gyro_y_bias; gyro_z_bias;... 
                    accel_x_bias; accel_y_bias; accel_z_bias;...
                    gyro_x_scale; gyro_y_scale; gyro_z_scale;... 
                    accel_x_scale; accel_y_scale; accel_z_scale;...
                    IMU_calib_a; IMU_calib_b; IMU_calib_c; IMU_calib_d;... 
                    f_x; f_y; o_x; o_y;...
                    cam_exts_a; cam_exts_b; cam_exts_c; cam_exts_d;...
                    cam_exts_x; cam_exts_y; cam_exts_z];
               
fixed_state_dot_list = [pos_north_dot; pos_east_dot; pos_down_dot;...
                        vn_dot; ve_dot; vd_dot; a_dot; b_dot; c_dot; d_dot;...
                        gyro_x_bias_dot; gyro_y_bias_dot; gyro_z_bias_dot;...
                        accel_x_bias_dot; accel_y_bias_dot; accel_z_bias_dot;...
                        gyro_x_scale_dot; gyro_y_scale_dot; gyro_z_scale_dot;... 
                        accel_x_scale_dot; accel_y_scale_dot; accel_z_scale_dot;...
                        IMU_calib_a_dot; IMU_calib_b_dot; IMU_calib_c_dot; IMU_calib_d_dot;... 
                        f_x_dot; f_y_dot; o_x_dot; o_y_dot;...
                        cam_exts_a_dot; cam_exts_b_dot; cam_exts_c_dot; cam_exts_d_dot;...
                        cam_exts_x_dot; cam_exts_y_dot; cam_exts_z_dot];


%--> Jacobian of the nonlinear system transition matrix (F_IMU)
for i = 1:length(fixed_state_list)
    for j = 1:length(fixed_state_list)
        F_fixed(i,j) = diff(fixed_state_dot_list(i), fixed_state_list(j));
    end
end


%--> Jacobian of the noise shaping vector (G)
for j = 1:length(fixed_state_list)
    G_fixed(j,1) = diff(fixed_state_dot_list(j), wg_noise);
end


% %--> Jacobian of the range and rangerate with respect to states 
% for i = 1:length(symbol_list)
%     H_row_range(1,i) = [diff(sat_range, symbol_list(i))];
% end
% 
% for i = 1:length(symbol_list)
%     H_row_rangeRate(1,i) = [diff(sat_rangeRate, symbol_list(i))];
% end


%--> Jacobian of the reprojection error with respect to alpha, beta, prho
sensitivity_list2 = [f_0_alpha; f_0_beta; f_0_prho];
for i = 1:length(r_j_i)
    for j = 1:length(sensitivity_list2)
        J_r_j_i(i,j) = diff(r_j_i(i, 1), sensitivity_list2(j, 1));
    end
end


%--> Jacobian of vision measurment with respect to involving fixed states
sensitivity_list3 = [f_x; f_y; o_x; o_y;...
                     cam_exts_a; cam_exts_b; cam_exts_c; cam_exts_d;...
                     cam_exts_x; cam_exts_y; cam_exts_z];
for i = 1:length(U_f_j_i)
    for j = 1:length(sensitivity_list3)
        H_cam_X_fixed_i(i,j) = diff(U_f_j_i(i,1), sensitivity_list3(j));
    end
end



%--> Jacobian of vision measurment with respect to the camera poses
sensitivity_list4 = [pos_north_i; pos_east_i; pos_down_i; att_a_i; att_b_i; att_c_i; att_d_i];
for i = 1:length(U_f_j_i)
    for j = 1:length(sensitivity_list4)
        H_cam_X_pi_i(i,j) = diff(U_f_j_i(i,1), sensitivity_list4(j));
    end
end

sensitivity_list5 = [q1;q2;q3;q4];
Euler_list = [Euler_phi_q;Euler_theta_q;Euler_psi_q];
for i = 1:length(Euler_list)
    for j = 1:length(sensitivity_list5)
        J_Euler_Q(i,j) = diff(Euler_list(i), sensitivity_list5(j));
    end
end
