%{
Particle Filter 2D motion Illustration Example
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
rng(0);

%--> Settings
Apply_PF = 1;
Use_noisy_sensors = 1;

%--> load data
load ('2D_trajectory_data');

%--> sampling period
dT = mean(diff(t));

%--> Set observation measurement vectors
y_observed = [North_GPS; East_GPS;Azimuth_Magnetometer];

%-> How spread our initial particle cloud
east_initialization_std = 2.0;
north_initialization_std = 2.0;
speed_initialization_std = 0.5;
azimuth_initialization_std = pi/10;
accel_bias_initialization_std = 0.6;
gyro_bias_initialization_std = pi/2;

%--> How noisy our system state prediction
east_prediction_std = 0.01;
north_prediction_std = 0.01;
speed_prediction_std = 0.01;
azimuth_prediction_std = 0.05;
accel_bias_prediction_std = 0.0;
gyro_bias_prediction_std = 0.0;

%--> Initialize Particles
system_order = 6;
Num_of_Particles = 500;
Num_of_Epochs = length(t);
X = zeros(Num_of_Epochs,Num_of_Particles , system_order);
weights = zeros(Num_of_Epochs,Num_of_Particles);
X(1,:,1) = North_ref(1) + east_initialization_std*randn(1,Num_of_Particles);
X(1,:,2) = East_ref(1) + north_initialization_std*randn(1,Num_of_Particles);
X(1,:,3) = Speed_ref(1) + speed_initialization_std*randn(1,Num_of_Particles);
X(1,:,4) = Azimuth_ref(1) + azimuth_initialization_std*randn(1,Num_of_Particles);
X(1,:,5) = 0.0 + accel_bias_initialization_std*randn(1,Num_of_Particles);
X(1,:,6) = 0.0 + gyro_bias_initialization_std*randn(1,Num_of_Particles);
weights(1,:) = 1/Num_of_Particles;

%--> Measurement Noise Covariance
R = diag([0.5 0.5 100]);

%--> Initialize State Covariances
P_pN = zeros(1,length(t));
P_pE = zeros(1,length(t));
P_v = zeros(1,length(t));
P_azimuth = zeros(1,length(t));
P_a = zeros(1,length(t));
P_g = zeros(1,length(t));

P_pN(1) = var(X(1,:,1));
P_pE(1) = var(X(1,:,2));
P_v(1) = var(X(1,:,3));
P_azimuth(1)  = var(X(1,:,4));
P_a(1) = var(X(1,:,5));
P_g(1) = var(X(1,:,6));

%--> Initial PF Solution State
North_PF = zeros(1,Num_of_Epochs);
East_PF = zeros(1,Num_of_Epochs);
Speed_PF = zeros(1,Num_of_Epochs);
Azimuth_PF  = zeros(1,Num_of_Epochs);
Acc_bias_PF = zeros(1,Num_of_Epochs);
Gyro_bias_PF = zeros(1,Num_of_Epochs);
North_PF(1) = mean(X(1,:,1));
East_PF(1) = mean(X(1,:,2));
Speed_PF(1) = mean(X(1,:,3));
Azimuth_PF(1)  = mean(X(1,:,4));
Acc_bias_PF(1) = mean(X(1,:,5));
Gyro_bias_PF(1) = mean(X(1,:,6));

%--> Main Loop
for k = 1:Num_of_Epochs-1
    %--> PF Prediction
    % Bias prediction (random constant process)
    X(k+1,:,5) = X(k,:,5) + accel_bias_prediction_std*randn(1,Num_of_Particles);
    X(k+1,:,6) = X(k,:,6) + gyro_bias_prediction_std*randn(1,Num_of_Particles);
    % Speed and Azimuth Prediction
    X(k+1,:,3) = X(k,:,3) + (acc_x_noisy(k+1) - X(k,:,5))*dT + speed_prediction_std*randn(1,Num_of_Particles);
    X(k+1,:,4) = X(k,:,4) + (gyro_z_noisy(k+1) - X(k,:,6))*dT + azimuth_prediction_std*randn(1,Num_of_Particles);
    % North and East Position Prediction
    X(k+1,:,1) = X(k,:,1) + X(k+1,:,3).*cos(X(k+1,:,4))*dT + north_prediction_std*randn(1,Num_of_Particles);
    X(k+1,:,2) = X(k,:,2) + X(k+1,:,3).*sin(X(k+1,:,4))*dT + east_prediction_std*randn(1,Num_of_Particles);
    
    %--> PF Update every one second
    if (mod(t(k),1) == 0 && t(k) > 0 && Apply_PF == 1 && ~(t(k)>= 10 && t(k) <= 12))
        for p = 1:Num_of_Particles
            % Calculate measurement error (y_observed-y_predicted)
            e_vector(1,1) = (y_observed(1,k+1)-X(k+1,p,1));
            e_vector(2,1) = (y_observed(2,k+1)-X(k+1,p,2));
            e_vector(3,1) = (angdiff(y_observed(3,k+1),X(k+1,p,4)));
            % Update weights
            weights(k+1,p) = 100*exp(-0.5*e_vector'/inv(R)*e_vector);
        end
        % Resample
        index = randsample(Num_of_Particles,Num_of_Particles,true,weights(k+1,:));
        X(k+1,:,:) = X(k+1,index,:);
        % Normalize weights
        weights(k+1,:) = 1/Num_of_Particles;
    end

    %--> keep the error state covariance diagnoal elements for plots
    P_pN(k+1) = var(X(k+1,:,1));
    P_pE(k+1) = var(X(k+1,:,2));
    P_v(k+1) = var(X(k+1,:,3));
    P_azimuth(k+1)  = var(X(k+1,:,4));
    P_a(k+1) = var(X(k+1,:,5));
    P_g(k+1) = var(X(k+1,:,6));

    North_PF(k+1) = mean(X(k+1,:,1));
    East_PF(k+1) = mean(X(k+1,:,2));
    Speed_PF(k+1) = mean(X(k+1,:,3));
    Azimuth_PF(k+1)  = mean(X(k+1,:,4));
    Acc_bias_PF(k+1) = mean(X(k+1,:,5));
    Gyro_bias_PF(k+1) = mean(X(k+1,:,6));

    fprintf('epoch %d/%d\n',k,Num_of_Epochs);

end

figure;
plot(t,X(:,:,4)*180/pi,'LineWidth',0.5);hold on;
plot(t,Azimuth_PF*180/pi,'LineWidth',3,'color','g');
plot(t,Azimuth_ref*180/pi,'b','LineWidth',3);grid on;
%plot(t,Azimuth_Magnetometer*180/pi,'k','LineWidth',2);
%plot(t,x(4,:)*180/pi,'r','LineWidth',5);
title('Azimth(degrees)');xlabel ('time(s)');ylabel('Azimuth(degrees)');
%legend('True Value','Noisy Azimuth','Prediction-Only','Particles');

figure;
plot(t,X(:,:,2),'LineWidth',0.5);hold on;
plot(t,East_PF,'LineWidth',3,'color','g');
plot(t,East_ref,'b','LineWidth',3);grid on;hold on;
%plot(t,East_GPS,'k*');
%plot(t,x(2,:),'r','linewidth',4);
title('East(m)');xlabel ('time(s)');ylabel('East(m)');
%legend('True Value','Noisy updates','KF');

figure;
plot(t,X(:,:,1),'LineWidth',0.5);hold on;
plot(t,North_PF,'LineWidth',3,'color','g');
plot(t,North_ref,'b','LineWidth',3);grid on;hold on;
%plot(t,North_GPS,'k*');
%plot(t,x(1,:),'r','linewidth',4);
title('North(m)');xlabel ('time(s)');ylabel('North(m)');

figure;%plot(t(1:100:end),X(1:100:end,:,5),'LineWidth',2);hold on;
plot(t,Acc_bias_PF,'LineWidth',3,'color','r');hold on;
plot(t,true_acc_x_bias*ones(1,length(t)),'-.b','LineWidth',2);grid on;
title('Accel x bias');xlabel ('time(s)');ylabel('Accelerometer Bias(m/s^2)');
legend('PF Estimated Bias','True Bias');

figure;%plot(t(1:100:end),X(1:100:end,:,6),'LineWidth',2);hold on;
plot(t,Gyro_bias_PF,'LineWidth',3,'color','r');hold on;
plot(t,true_gyro_z_bias*ones(1,length(t)),'-.b','LineWidth',2);grid on;
title('Gyro z bias');xlabel ('time(s)');ylabel('Gyro-Z Bias(r/s)');
legend('PF Estimated Bias','True Bias');

figure;
Azimuth_error_vector = angdiff(Azimuth_ref,Azimuth_PF)*180/pi;
Vertices(:,2) = [Azimuth_error_vector' + 1*sqrt(P_azimuth'); flipdim( Azimuth_error_vector' - 1*sqrt(P_azimuth'),1)];
Vertices(:,1) = [t'; flipdim(t',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(t,Azimuth_error_vector,'LineWidth',2);
title('Azimth Error(degrees)');xlabel ('time(s)');ylabel('Azimuth Error(degrees)');grid on;
legend('STDV (1 \sigma)','Error(deg)');

figure;
East_error_vector = East_ref-East_PF;
Vertices(:,2) = [East_error_vector' + 1*sqrt(P_pE'); flipdim( East_error_vector' - 1*sqrt(P_pE'),1)];
Vertices(:,1) = [t'; flipdim(t',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(t,(East_ref-East_PF),'LineWidth',2);ylim([-10 10]);
title('East Error(m)');xlabel ('time(s)');ylabel('East Error(m)');grid on;
legend('STDV (1 \sigma)','Error(m)');

figure;
North_error_vector = North_ref-North_PF;
Vertices(:,2) = [North_error_vector' + 1*sqrt(P_pE'); flipdim( North_error_vector' - 1*sqrt(P_pE'),1)];
Vertices(:,1) = [t'; flipdim(t',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(t,(North_ref-North_PF),'LineWidth',2);ylim([-10 10]);
title('North Error(m)');xlabel ('time(s)');ylabel('North Error(m)');grid on;
legend('STDV (1 \sigma)','Error(m)');

fprintf('heading error = %f\n',sqrt(mean(angdiff(Azimuth_ref,Azimuth_PF)*180/pi).^2));
fprintf('East Position error = %f\n',sqrt(mean(East_ref-East_PF).^2));
fprintf('North error = %f\n',sqrt(mean(North_ref-North_PF).^2));

return;

figure;plot(t,P_azimuth,'r');hold on;plot(t,P_g,'b');grid on;
title('Error Covariance');xlabel ('time(s)');ylabel('Variance(m2)');
legend('Azimuth','Gyro-z bias');

figure;plot(t,acc_x_noisy,'r');hold on;plot(t,acc_x_clean,'b');grid on;
title('Acceleration control signal');xlabel ('time(s)');ylabel('Acceleration control signal(m/s2)');
legend('Noisy Biased Input','True Correct Input');

figure;plot(t,gyro_z_noisy,'r');hold on;plot(t,gyro_z_clean,'b');grid on;
title('Steering control');xlabel ('time(s)');ylabel('Steering control(rad/s)');
legend('Noisy Biased Input','True Correct Input');

figure;plot(t,P_pE,'r');hold on;plot(t,P_pN,'b');grid on;
title('Error Covariance');xlabel ('time(s)');ylabel('Variance(m2)');
legend('East','North');


figure;
plot(y_observed(2,:),y_observed(1,:),'*k');hold on;grid on;
plot(x(2,:),x(1,:),'r','linewidth',3);
plot(East_ref,North_ref,'b','linewidth',3);
legend('Noisy Measurements','EKF','Ground Truth');
title('2D position(m)');xlabel ('East(m)');ylabel('North(m)');

% figure;plot(t,f_a_true,'b');grid on;hold on;
% plot(t,f_a_noisy,'k','linewidth',1);
% plot(t,x(6,:),'r','linewidth',1);
% title('Forard Acceleration Control (m/s3)');xlabel ('time(s)');ylabel('Acc Control(m/s3)');
% legend('True Value','Noisy Inout','EKF estimate');
% 
% figure;plot(t,f_theta_true,'b');grid on;hold on;
% plot(t,f_theta_noisy,'k','linewidth',1);
% plot(t,x(7,:),'r','linewidth',1);
% title('Azimuth Control Inout (rad/s)');xlabel ('time(s)');ylabel('Angle Control(rad/s)');
% legend('True Value','Noisy Inout','EKF estimate');
