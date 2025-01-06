%{
Kalman Filter 2D motion Illustration Example
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

set(0,'DefaultFigureWindowStyle','docked');
close all;
clear;
clc;

load ('2D_trajectory_data');

Apply_EKF = 1;
Use_noisy_input = 1;
acc_x_noise_std = 0.0059; 
gyro_z_noise_std = 0.1*pi/180;
pos_meas_noise_std = 10; % common horizontal accuracy of standard GPS
heading_meas_noise_std = 5*pi/180; % common accuracy of magnetic sensor heading 
y_observed = [North_GPS; East_GPS;Azimuth_Magnetometer];

%-> State Vector Initialization
x(1,1) = North_ref(1);
x(2,1) = East_ref(1);
x(3,1) = Speed_ref(1);
x(4,1) = Azimuth_ref(1);
x(5,1) = 0.0; % gyro-Z bias
x(6,1) = 0.0; % accel-x (forward) bias

%--> The system (transition) matrix F
F = [0 0 cos(x(4,1)) -x(3,1)*sin(x(4,1))  0 0;
     0 0 sin(x(4,1))  x(3,1)*cos(x(4,1))  0 0;
     0 0 0            0                  -1 0;
     0 0 0            0                  0 -1;
     0 0 0            0                  0 0;
     0 0 0            0                  0 0;
    ];

%--> Design matrux H
H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 0 1 0 0];

G = diag([0.1 0.1 0.1 0.1 acc_x_noise_std^2 gyro_z_noise_std^2]);
Q = diag([1 1 1 1 1 1]);

R = diag([pos_meas_noise_std^2 pos_meas_noise_std^2 heading_meas_noise_std^2]);

P(:,:,1) = diag([25; 25; 10; 1e-03; 5; 5]);

dT = mean(diff(t));

P_pN = zeros(1,length(t));
P_pE = zeros(1,length(t));
P_v = zeros(1,length(t));
P_azimuth = zeros(1,length(t));
P_a = zeros(1,length(t));

P_pN(1) = P(1,1,1);
P_pE(1) = P(2,2,1);
P_v(1) = P(3,3,1);
P_azimuth(1)  = P(4,4,1);
P_fa(1) = P(5,5,1);
P_ftheta(1) = P(6,6,1);

system_order = 6;
H_history = zeros(size(H));
Phi_history = zeros(size(F));
observability_counter = 1;
observability_record_counter = 1;

for k = 1:length(t)-1
    %--> KF Prediction
    %State Prediction
    error_states = zeros(system_order,1);
    x(5,k+1) = x(5,k);
    x(6,k+1) = x(6,k);
    if Use_noisy_input == 1
        x(3,k+1) = x(3,k) + (acc_x_noisy(k+1) - x(5,k))*dT;% forward velocity prediction
        x(4,k+1) = x(4,k) + (gyro_z_noisy(k+1) - x(6,k))*dT;% azimuth prediction
   else
        x(3,k+1) = x(3,k) + (acc_x_clean(k+1) - x(5,k+1))*dT;% forward acceleration prediction
        x(4,k+1) = x(4,k) + (gyro_z_clean(k+1) - x(6,k+1))*dT;% azimuth prediction
    end
    x(1,k+1) = x(1,k) + x(3,k+1)*cos(x(4,k+1))*dT; % north position prediction
    x(2,k+1) = x(2,k) + x(3,k+1)*sin(x(4,k+1))*dT; % east position prediction
    
    F = [0 0 cos(x(4,k)) -x(3,k)*sin(x(4,k))  0 0;
         0 0 sin(x(4,k))  x(3,k)*cos(x(4,k))  0 0;
         0 0 0            0                  -1 0;
         0 0 0            0                  0 -1;
         0 0 0            0                  0 0;
         0 0 0            0                  0 0;
        ];

    Phi = eye(system_order,system_order) + F*dT;

    % Calculate System Noise Matrix
    Qd = dT^2*G*Q*G';
    
    % Predict State Error Covariance
    P(:,:,k+1) = Phi*P(:,:,k)*Phi' + Qd;
    
    %--> KF Update every one second, apply outage between second 30 and 50
    if (mod(t(k),0.01) == 0 && t(k) > 0 && Apply_EKF == 1 && ~(t(k)>= 30 && t(k) <= 50))
        % Calculate Kalman Gain
        K = P(:,:,k+1)*H'/(H*P(:,:,k+1)*H'+R);
        % Update error covariance matrix P
        P(:,:,k+1) = P(:,:,k+1) - K*H*P(:,:,k+1);
        % Calculate error state
        error_states = K*(y_observed(:,k+1) - H*x(:,k+1));
        % Correct states
        x(:,k+1) = x(:,k+1) + error_states;
    end
    % Normalize P
    P(:,:,k+1) = (P(:,:,k+1)+P(:,:,k+1))/2;
    %--> keep the error state covariance diagnoal elements for plots
    P_pN(k+1) = P(1,1,k+1);
    P_pE(k+1) = P(2,2,k+1);
    P_v(k+1) = P(3,3,k+1);
    P_azimuth(k+1)  = P(4,4,k+1);
    P_fa(k+1) = P(5,5,k+1);
    P_ftheta(k+1) = P(6,6,k+1);
end

figure;plot(t(1:1500),x(5,1:1500),'-r','LineWidth',2);hold on;plot(t(1:1500),true_acc_x_bias*ones(1,length(t(1:1500))),'--','color','b','LineWidth',2);grid on;
title('Acc x bias convergence');xlabel ('time(s)');ylabel('Steering control bias(m/s2)');
legend('KF','True Value');

figure;plot(t(1:600),x(6,1:600),'-r','LineWidth',2);hold on;plot(t(1:600),true_gyro_z_bias*ones(1,length(t(1:600))),'--','color','b','LineWidth',2);grid on;
title('Gyro z bias convergence');xlabel ('time(s)');ylabel('Acceleration control signal bias(rad/s)');
legend('KF','True Value');

figure;
plot(t,Azimuth_ref*180/pi,'b');grid on;hold on;
plot(t(1:10:end),Azimuth_Magnetometer(1:10:end)*180/pi,'k*');
plot(t,x(4,:)*180/pi,'-.r','LineWidth',2);
title('Azimth(degrees)');xlabel ('time(s)');ylabel('Azimuth(degrees)');
legend('Ground Truth','Noisy Measurements','EKF Estimation');

figure;plot(t,East_ref,'b');grid on;hold on;
plot(t(1:10:end),East_GPS(1:10:end),'k*');
plot(t,x(2,:),'r','linewidth',4);
title('East(m)');xlabel ('time(s)');ylabel('East(m)');
legend('Ground Truth','Noisy Measurements','EKF Estimation');

figure;plot(t,North_ref,'b');grid on;hold on;
plot(t(1:10:end),North_GPS(1:10:end),'k*');
plot(t,x(1,:),'r','linewidth',4);
title('North(m)');xlabel ('time(s)');ylabel('North(m)');
legend('Ground Truth','Noisy Measurements','EKF Estimation');

figure;
plot(t,acc_x_noisy,'r');hold on;plot(t,acc_x_clean,'b');grid on;
title('Accelerometer-X');xlabel ('time(s)');ylabel('Accel(m/s2)');
legend('Noisy Biased','True Acceleration');

figure;plot(t,gyro_z_noisy,'r');hold on;plot(t,gyro_z_clean,'b');grid on;
title('Gyroscope-Z');xlabel ('time(s)');ylabel('Gyro(rad/s)');
legend('Noisy Biased','True Turn Rate');

figure;plot(t,abs(acc_x_clean-acc_x_noisy),'r');grid on;ylim([0.4 0.6]);
title('Added Accelerometer-X Errors');xlabel ('time(s)');ylabel('Accel Error(m/s2)');

figure;plot(t,abs(gyro_z_clean-gyro_z_noisy),'r');grid on;ylim([0.4 0.6]);
title('Added Gyroscope-Z Errors');xlabel ('time(s)');ylabel('(rad/s)');

figure;plot(t,P_pE,'r');hold on;plot(t,P_pN,'b');grid on;
title('Error Covariance');xlabel ('time(s)');ylabel('Variance(m2)');
legend('East','North');

figure;
Vertices(:,2) = [x(5,:)' + 1*sqrt(P_fa'); flipdim( x(5,:)' - 1*sqrt(P_fa'),1)];
Vertices(:,1) = [t'; flipdim(t',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(t,x(5,:),'r','LineWidth',1);hold on;
plot(t,true_acc_x_bias*ones(1,length(t)),'-.b','LineWidth',2);grid on;
title('Accel x bias');xlabel ('time(s)');ylabel('Accelerometer Bias(m/s^2)');
legend('STDV (1 \sigma)','EKF Estimated Bias','True Bias');

figure;
Vertices(:,2) = [x(6,:)' + 1*sqrt(P_fa'); flipdim( x(6,:)' - 1*sqrt(P_fa'),1)];
Vertices(:,1) = [t'; flipdim(t',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(t,x(6,:),'r','LineWidth',1);hold on;
plot(t,true_gyro_z_bias*ones(1,length(t)),'-.b','LineWidth',2);grid on;
title('Gyro Z bias');xlabel ('time(s)');ylabel('Gyro Bias(rad/s)');
legend('STDV (1 \sigma)','EKF Estimated Bias','True Bias');

figure;
plot(y_observed(2,:),y_observed(1,:),'*k');hold on;grid on;
plot(x(2,:),x(1,:),'r','linewidth',3);
plot(East_ref,North_ref,'b','linewidth',3);
legend('Noisy Measurements','EKF','Ground Truth');
title('2D position(m)');xlabel ('East(m)');ylabel('North(m)');

heading_error_vector = angdiff(Azimuth_ref,x(4,:));

figure;
Vertices(:,2) = 180/pi*[heading_error_vector' + 1*sqrt(P_azimuth'); flipdim( heading_error_vector' - 1*sqrt(P_azimuth'),1)];
Vertices(:,1) = [t'; flipdim(t',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(t,180/pi*heading_error_vector,'LineWidth',2);
title('Azimth Error(degrees)');xlabel ('time(s)');ylabel('Azimuth Error(degrees)');grid on;
legend('STDV (1 \sigma)','Error(deg)');

east_error_vector = (East_ref-x(2,:));

figure;
Vertices(:,2) = [east_error_vector' + 1*sqrt(P_pE'); flipdim( east_error_vector' - 1*sqrt(P_pE'),1)];
Vertices(:,1) = [t'; flipdim(t',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(t,east_error_vector,'LineWidth',2);
title('East Error(m)');xlabel ('time(s)');ylabel('East Error(m)');grid on;
legend('STDV (1 \sigma)','Error(m)');

north_error_vector = (North_ref-x(1,:));

figure;
Vertices(:,2) = [north_error_vector' + 1*sqrt(P_pE'); flipdim( north_error_vector' - 1*sqrt(P_pE'),1)];
Vertices(:,1) = [t'; flipdim(t',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(t,north_error_vector,'LineWidth',2);
title('North Error(m)');xlabel ('time(s)');ylabel('North Error(m)');grid on;
legend('STDV (1 \sigma)','Error(m)');

fprintf('heading error = %f\n',sqrt(mean(angdiff(Azimuth_ref,x(4,:))*180/pi).^2));
fprintf('East Position error = %f\n',sqrt(mean(East_ref-x(2,:)).^2));
fprintf('North error = %f\n',sqrt(mean(North_ref-x(1,:)).^2));

