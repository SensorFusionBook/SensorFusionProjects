%{
LiDAR ICP Algorithm Illustration Example
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

ACKNOWLEGEMNTs: This project uses the following ICP algorithm implementation
https://www.mathworks.com/matlabcentral/fileexchange/27804-iterative-closest-point
And the InterX MATLAB function to find intersection of two curves 

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

set(0,'DefaultFigureWindowStyle','docked');
close all;
clear;
clc;

load 'simulated_lidar_data_noise_free.mat';
%load 'simulated_lidar_data_noisy.mat';

Alpha = 1.5;           %LiDAR sensor angular resolution
LidarFOV = [-20 200];  %LiDAR field of view

[N,L] = size(scan);

i = 140;
j = 160;

figure;
ref_scan_x = 0;
ref_scan_y = 0;

for k = 1:L
    beam_angle = LidarFOV(1) + (k-1)*Alpha;       
    scan_point(1) = scan(i,k)*cos(beam_angle*pi/180);
    scan_point(2) = scan(i,k)*sin(beam_angle*pi/180);
    ref_scan_x = [ref_scan_x scan_point(1)];
    ref_scan_y = [ref_scan_y scan_point(2)];
    M(1,k) = scan_point(1);
    M(2,k) = scan_point(2);
    M(3,k) = 0;    
end
plot(ref_scan_x,ref_scan_y,'*','color','k','linewidth',4);hold on;

axis equal;

cur_scan_x = 0;
cur_scan_y = 0;
for k = 1:L
    beam_angle = LidarFOV(1) + (k-1)*Alpha;       
    scan_point(1) = scan(j,k)*cos(beam_angle*pi/180);
    scan_point(2) = scan(j,k)*sin(beam_angle*pi/180);
    cur_scan_x = [cur_scan_x scan_point(1)];
    cur_scan_y = [cur_scan_y scan_point(2)];
    D(1,k) = scan_point(1);
    D(2,k) = scan_point(2);
    D(3,k) = 0;    
end
plot(cur_scan_x,cur_scan_y,'o','color','b','MarkerSize',16);hold on;

dA = (A(j)-A(i))*pi/180;
A = A(i)*pi/180;
dR = [cos(dA) sin(dA);-sin(dA) cos(dA)];
R_l2r = [cos(A) -sin(A);sin(A) cos(A)];
dT = R_l2r*[x(j) - x(i); y(j) - y(i)] ;

cur_transformed_scan_x = dT(1);
cur_transformed_scan_y = dT(2);

for k = 1:L
    beam_angle = LidarFOV(1) + (k-1)*Alpha;       
    scan_point(1) = scan(j,k)*cos(beam_angle*pi/180);
    scan_point(2) = scan(j,k)*sin(beam_angle*pi/180);
    scan_point = dR*[scan_point(1);scan_point(2)] + dT;
    cur_transformed_scan_x = [cur_transformed_scan_x scan_point(1)];
    cur_transformed_scan_y = [cur_transformed_scan_y scan_point(2)];
end
plot(cur_transformed_scan_x,cur_transformed_scan_y,'diamond','color','r','MarkerSize',16);hold on;
legend('reference scan','current scan','trasformed current scan');

xlabel('x(m)');
ylabel('y(m)');
title('lidar scan transformation (2-D example)');
grid on;

fprintf('True dA = %f (deg), dT = [%f,%f](m)\n', dA*180/pi, dT(1), dT(2));

% estimate dR and dT using ICP scan-matching

[Ricp Ticp ER t correspondences] = icp(M, D, 40);

dR = Ricp(1:2,1:2);
dA = atan2(dR(1,2),dR(1,1));
dT = [Ticp(1);Ticp(2)];

fprintf('ICP dA = %f (deg), dT = [%f,%f](m)\n', dA*180/pi, dT(1), dT(2));

cur_transformed_scan_x = dT(1);
cur_transformed_scan_y = dT(2);

for k = 1:L
    beam_angle = LidarFOV(1) + (k-1)*Alpha;       
    scan_point(1) = scan(j,k)*cos(beam_angle*pi/180);
    scan_point(2) = scan(j,k)*sin(beam_angle*pi/180);
    scan_point = dR*[scan_point(1);scan_point(2)] + dT;
    cur_transformed_scan_x = [cur_transformed_scan_x scan_point(1)];
    cur_transformed_scan_y = [cur_transformed_scan_y scan_point(2)];
end
plot(cur_transformed_scan_x,cur_transformed_scan_y,'<','color','g','MarkerSize',16);hold on;

for k = 1:L
    plot([M(1,correspondences(k)) D(1,k)],[M(2,correspondences(k)) D(2,k)]);
end

legend('Reference scan','Current scan','True transformed scan','ICP transformed scan');

figure;
plot(ER,'*','linewidth',2);grid on;xlabel('iteration');ylabel('J(R,T,M');title('cost function convergence');