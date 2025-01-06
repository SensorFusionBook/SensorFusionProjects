%{
8-Points Algorithm for Camera Motion Estimation
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

Acknowledgements
The data used in this project is based on the MATLAB camera calibration example in the following links:
https://www.mathworks.com/help/vision/camera-calibration.html
https://www.mathworks.com/help/vision/ug/evaluating-the-accuracy-of-single-camera-calibration.html

%}

set(0,'DefaultFigureWindowStyle','docked');
clear;clc;close all;

%--> Ensure the reproducibility of the randomness in MATLAB built-in codes
rng(0);

%--> Load data
load ('eight-pts-algorithm_data.mat');

point_index = 1:1:54; % Points on the board
camera_pose_indx1 = 1; % First Camera Pose
camera_pose_indx2 = 7; % Second Camera Pose

%--> Pixel Points
q = imagePoints(point_index,:,camera_pose_indx1);
p = imagePoints(point_index,:,camera_pose_indx2);

%--> Camera Intrinsics Caliration Parameters
K = params.Intrinsics.K;

%--> Estimate relative rotation and translation using MATLAB builtin functions
F  = estimateFundamentalMatrix(p,q);
E = K'*F*K;
[U, S, V] = svd(E,"econ");
E = U*diag([(S(1,1)+S(2,2))/2 (S(1,1)+S(2,2))/2 0])*V';
relativePose = estrelpose(E,params.Intrinsics,p,q); 
R1 = relativePose.R;
T1 = relativePose.Translation;
[roll,pitch,Azimuth] = dcm2angle(R1,'XYZ');
fprintf('The MATLAB  estrelpose Results  : roll,pitch,Azimuth = %f,%f,%f, T = [%f,%f,%f]\n',roll*180/pi,pitch*180/pi,Azimuth*180/pi,T1);

%--> Estimate relative rotation and translation using the 8 Points Algorithm
q = [q ones(length(q),1)]';
p = [p ones(length(p),1)]';

%--> 8-points algorithms (inputs: matching points q,p and intrinsics K)

% Get normalized hmogeneous coordinates from pixel coordinates and K matrix
q_ = K\q;
p_ = K\p;

% Form the A matrix using the Kronecker product
A = zeros(length(p),9);
for i = 1:length(p)
    x1 = p_(1,i);
    y1 = p_(2,i);
    x2 = q_(1,i);
    Y2 = q_(2,i);
    A(i,:) = [x2*x1 x2*y1 x2 Y2*x1 Y2*y1 Y2 x1 y1 1];
end

%Find the essential matrix vector using SVD of A
[~, ~, V] = svd(A);
e = V(:,9);
F = reshape(e, 3, 3);

% Refine the essential matrix using SVD
[U, S, V] = svd(F,"econ");
E = U*diag([(S(1,1)+S(2,2))/2 (S(1,1)+S(2,2))/2 0])*V';

% Get the 4 possible solutions from the Essential matrix E
Ts = get4PossibleSolutions(E);

% Select the solution that assures positive depth
Ts = SelectCorrectCameraTransformationMatrix(p,q,Ts,K);
R2 = Ts(:,1:3);
T2 = Ts(:,4);

% Display the orientation in Euler angles and the Translation vector 
[roll,pitch,Azimuth] = dcm2angle(R2,"XYZ");
fprintf('The 8 Points Algorithm Results  : roll,pitch,Azimuth = %f,%f,%f, T = [%f,%f,%f]\n',roll*180/pi,pitch*180/pi,Azimuth*180/pi,T2);

% Calculate the angle between the two rotations R1 and R2
R_relative = R1' * R2;
trace_R = trace(R_relative);
theta = acos((trace_R - 1) / 2);
theta = rad2deg(theta);
fprintf('The rotation angle between R1 and R2 is: %.4f degrees\n', theta);