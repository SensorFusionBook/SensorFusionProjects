%{
Demonstration of Allan Variance
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

clear;
close all;
fclose all;
clc;
set(0,'DefaultFigureWindowStyle','docked');

D2R = pi/180;
R2D = 180/pi;

%--> Load data sets of all IMU grades
%--> Gyro is in deg/s and accel is in m/s2

load imu_stationary_data_sets\MPU9250.mat;
load imu_stationary_data_sets\MTi100.mat;

%--> Prepare the time vector

numSamples = length(MPU9250_accel_Y);
Fs = 100;
t0 = 1/Fs;
t = (0:(numSamples-1))/Fs;

% Plot raw data

figure;
plot(MPU9250_accel_Y);grid on;ylabel('m/s2');xlabel('time');
title('MPU 9250 Accel Y data');
figure;
plot(MPU9250_gyro_X);grid on;ylabel('deg/s');xlabel('time');
title('MPU 9250 Gyro X data');
figure;
plot(MTi100_accel_Y);grid on;ylabel('m/s2');xlabel('time');
title('MTi100 Accel Y data');
figure;
plot(MTi100_gyro_X);grid on;ylabel('deg/s');xlabel('time');
title('MTi100 Gyro X data');

%--> Apply Allan Variance on GyroX of all grades, plot them on one figure;

[avar,tau] = allanvar(MTi100_gyro_X,'octave',Fs);
figure;loglog(tau,sqrt(avar),'b','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)')
title('Allan Deviation - Gyro(deg/sec)');grid on
MTi100_arw_gyro_x = find_av_parameter(tau,sqrt(avar),-0.5,1)

[avar,tau] = allanvar(MPU9250_gyro_X,'octave',Fs);
adev = sqrt(avar);loglog(tau,sqrt(avar),'r','LineWidth',2)
xlabel('\tau');ylabel('\sigma(\tau)')
legend('MTi-100','MPU 9250');grid on
MPU9250_arw_gyro_x = find_av_parameter(tau,sqrt(avar),-0.5,1)

%--> Apply Allan Variance on AccelY of all grades, plot them on one figure;

[avar,tau] = allanvar(MTi100_accel_Y,'octave',Fs);
adev = sqrt(avar);figure;loglog(tau,sqrt(avar),'b','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)')
title('Allan Deviation - Acc(m/sec2)');grid on
MTi100_vrw_acc_x = find_av_parameter(tau,sqrt(avar),-0.5,1)

[avar,tau] = allanvar(MPU9250_accel_Y,'octave',Fs);
adev = sqrt(avar);loglog(tau,sqrt(avar),'r','LineWidth',2)
xlabel('\tau');ylabel('\sigma(\tau)')
legend('MTi-100','MPU 9250');grid on
MPU9250_vrw_acc_x = find_av_parameter(tau,sqrt(avar),-0.5,1)
