%{
Imu Gps Fusion Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

function [trajectory_data] = InverseKinematics(trajectory_data, gyroNoise, gyroBias, gyroSF, accelNoise, accelBias, accelSF, posNoise, velNoise)
    
    %% defining constants
    dt = (trajectory_data.time(2)-trajectory_data.time(1));	% time step
    f = 1/dt;                                   % IMU sampling frequency
    we = 2*pi/(24*60*60);                       % earth rotation rate 
    T = length(trajectory_data.time)-1;                % number of samples or length of experiment
    D2R = pi/180;
    R2D = 180/pi;
    
    %% Inverse Kinematics   
    a = 6378137;                % Semi Major Axis of the Earth (in metres).
    b = 6356752.3142;           % Semi Minor Axis of the Earth (in metres).
    e2 = 1-(b^2)/(a^2);         % used in calculating radii of curvature (N & M).
    phi = trajectory_data.lat(1:T);
    Rn = a./sqrt(1-e2.*sin(phi).*sin(phi));
    Rm = (a.*(1-e2))./((1-e2.*sin(phi).*sin(phi)).^(1.5));


    % g_o = 9.780318 * ( 1 + 5.3024e-03.*sin(trajectory_data.lat(1:T)).^2 - 5.9e-06.*(sin(2*trajectory_data.lat(1:T))).^2 );
    % g = (g_o ./ (1 + (trajectory_data.h(1:T) ./ sqrt(Rn.*Rm))).^2);

    g = 9.8*ones(1,T);

    vL_dot = diff([trajectory_data.vel(:,1)';...
                   trajectory_data.vel(:,2)';...
                   trajectory_data.vel(:,3)'],1,2)./dt;        % discrete time derivative of vL


    vL_dot = vL_dot - [zeros(1,T);zeros(1,T);g];                          % adding the effect of gravity on accelerometer measurments

    wL_IE = zeros(3,length(vL_dot));                    % earth rotation rate projected on L frame                    
    wL_IE(1,:) =  we.*cos(trajectory_data.lat(1:T));
    wL_IE(3,:) = -we.*sin(trajectory_data.lat(1:T));     


    wL_EL = zeros(3,length(vL_dot));                    % Rotation casued by motion on ellipsoid model of earth 
    wL_EL(1,:) = trajectory_data.vel(1:T,2)./(Rn+trajectory_data.h(1:T));
    wL_EL(2,:) = -trajectory_data.vel(1:T,1)./(Rm+trajectory_data.h(1:T));
    wL_EL(3,:) = (-tan(trajectory_data.lat(1:T)).*trajectory_data.vel(1:T,2))./(Rn+trajectory_data.h(1:T));    

    vL_dot = vL_dot + cross(((wL_EL + 2.*wL_IE)),[trajectory_data.vel(1:T,1)';trajectory_data.vel(1:T,2)';trajectory_data.vel(1:T,3)']);    % Corriolis effect. acceleration caused by earth rotation & transport rate                            % Adding up all accelerations                              

    aB = zeros(3,T);                                    % Acceleration in body frame
    CL_B = zeros(3,3,T+1);                              % Direction Cosine Matrix (DCM)
    for i=1:T
        C_BL_value = angle2dcm(trajectory_data.heading(i),trajectory_data.pitch(i),trajectory_data.roll(i),'ZYX');
        CL_B(:,:,i) = C_BL_value';
        aB(:,i) = C_BL_value*vL_dot(:,i);
    end

    S = zeros(3,3,T);
    wB_LB = zeros(3,T);
    wB = zeros(3,T);                                    % Angular velocity in body frame
    for i=1:T
        S(:,:,i) = (CL_B(:,:,i)' * CL_B(:,:,i+1))-eye(3,3);
        wB_LB(:,i) = [S(3,2,i); S(1,3,i); S(2,1,i)]./dt;
        wB(:,i) = wB_LB(:,i) + (CL_B(:,:,i)' * (wL_EL(:,i) + wL_IE(:,i)));  % Including earth rotation and transport rate on gyroscope measurments
    end
    
    trajectory_data.gyro_clean = sgolayfilt(wB', 5, 99);
    trajectory_data.gyro_clean = [trajectory_data.gyro_clean;trajectory_data.gyro_clean(end,:)];
    trajectory_data.accel_clean = sgolayfilt(aB', 5, 99);
    trajectory_data.accel_clean = [trajectory_data.accel_clean; trajectory_data.accel_clean(end,:)];
    
    %% adding noise data
    % applying scale factor and bias happens after accelerometer and
    % gyrscope data have been contaminated with corriolis effect.
    % scale factor contamination is multiplicative and bias is additive
    
    trajectory_data.gyro_noisy = (trajectory_data.gyro_clean .* (1+gyroSF)) + (gyroBias*D2R) + ((gyroNoise*D2R) .* randn(size(trajectory_data.gyro_clean)));    
    trajectory_data.accel_noisy = (trajectory_data.accel_clean .* (1+accelSF)) + accelBias + (accelNoise .* randn(size(trajectory_data.accel_clean)));

    trajectory_data.gyro_noisy = sgolayfilt(trajectory_data.gyro_noisy, 10, 45);
    trajectory_data.accel_noisy = sgolayfilt(trajectory_data.accel_noisy, 10, 55);
    
    % Take the noisy IMU from ems simulated IMU
    %ems_data.gyro_noisy = ems_data.sim_gyro;
    %ems_data.accel_noisy = ems_data.sim_accel;
    
    trajectory_data.east_noisy = trajectory_data.east + posNoise(2) .* randn(size(trajectory_data.east));
    trajectory_data.north_noisy =  trajectory_data.north + posNoise(1) .* randn(size(trajectory_data.north));
    trajectory_data.h_noisy = trajectory_data.h + posNoise(3) .* randn(size(trajectory_data.h)) ;

    trajectory_data.vel_noisy = trajectory_data.vel + velNoise .* randn(size(trajectory_data.vel));
    trajectory_data.vel_N = [trajectory_data.vel(:,2), trajectory_data.vel(:,1), -trajectory_data.vel(:,3)];
    trajectory_data.vel_N_noisy = [trajectory_data.vel_noisy(:,2), trajectory_data.vel_noisy(:,1), -trajectory_data.vel_noisy(:,3)];    
    
    trajectory_data.noiseInfo.accel_bias = accelBias;
    trajectory_data.noiseInfo.accel_noise = accelNoise;
       
    trajectory_data.noiseInfo.gyro_bias = gyroBias;
    trajectory_data.noiseInfo.gyro_noise = gyroNoise;
    
    trajectory_data.noiseInfo.position_NED_noise = posNoise;
    trajectory_data.noiseInfo.vel_NED_noise = velNoise;
    
end

