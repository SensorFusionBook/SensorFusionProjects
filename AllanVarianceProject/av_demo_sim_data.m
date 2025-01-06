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
%--> Gyro is in rad/s and accel is in m/s2
load imu_stationary_data_sets\consumer_grade_imu_stationary_data.mat;
load imu_stationary_data_sets\industrial_grade_imu_stationary_data.mat;
load imu_stationary_data_sets\tactical_grade_imu_stationary_data.mat;
load imu_stationary_data_sets\navigation_grade_imu_stationary_data.mat;

%--> Prepare the time vector
numSamples = length(ConsumerGradeIMU_Acc(:,1));
Fs = 100;
t0 = 1/Fs;
t = (0:(numSamples-1))/Fs;

%--> Plot all the raw data (optional)
plot_imu_data(t,ConsumerGradeIMU_Acc,ConsumerGradeIMU_Gyr*R2D,'Consumer Grade IMU');
plot_imu_data(t,IndustrialGradeIMU_Acc,IndustrialGradeIMU_Gyr*R2D,'Industrial Grade IMU');
plot_imu_data(t,TacticalGradeIMU_Acc,TacticalGradeIMU_Gyr*R2D,'Tactical Grade IMU');
plot_imu_data(t,NavigationGradeIMU_Acc,NavigationGradeIMU_Gyr*R2D,'Navigation Grade IMU');

%--> Apply Allan Variance on GyroX of all grades, plot them on one figure;
figure;title('Allan Deviation - GyroX(deg/sec)')
[avar,tau] = allanvar(ConsumerGradeIMU_Gyr(:,1),'octave',Fs);
loglog(tau,sqrt(avar),'r','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)');grid on;

[avar,tau] = allanvar(IndustrialGradeIMU_Gyr(:,1),'octave',Fs);
loglog(tau,sqrt(avar),'g','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)');grid on;

[avar,tau] = allanvar(TacticalGradeIMU_Gyr(:,1),'octave',Fs);
loglog(tau,sqrt(avar),'b','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)');grid on;

[avar,tau] = allanvar(NavigationGradeIMU_Gyr(:,1),'octave',Fs);
loglog(tau,sqrt(avar),'k','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)');grid on;

legend('consumer grade gyro','industrial grade gyro','tactical grade gyro','navigation grade gyro');

%--> Apply Allan Variance on AccX of all grades, plot them on one figure;
figure;title('Allan Deviation - AccX(deg/sec)')
[avar,tau] = allanvar(ConsumerGradeIMU_Acc(:,1),'octave',Fs);
loglog(tau,sqrt(avar),'r','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)');grid on;

[avar,tau] = allanvar(IndustrialGradeIMU_Acc(:,1),'octave',Fs);
loglog(tau,sqrt(avar),'g','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)');grid on;

[avar,tau] = allanvar(TacticalGradeIMU_Acc(:,1),'octave',Fs);
loglog(tau,sqrt(avar),'b','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)');grid on;

[avar,tau] = allanvar(NavigationGradeIMU_Acc(:,1),'octave',Fs);
loglog(tau,sqrt(avar),'k','LineWidth',2);hold on;
xlabel('\tau');ylabel('\sigma(\tau)');grid on;

legend('consumer grade acc','industrial grade acc','tactical grade acc','navigation grade acc');