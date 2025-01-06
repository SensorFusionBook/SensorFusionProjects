%{
LiDAR odometry Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

%--> Initial Settings
set(0,'DefaultFigureWindowStyle','docked');
clear;clc;close all;

%--> Load data
load ('simulated_lidar_data.mat');
Num_of_epochs = length(t);

%--> LiDAR sensor parameters
Alpha = 1.5;                                %angular resolution in degrees
LidarFOV = [-20 200];                       %Lidar Field of View in degrees
L = round((LidarFOV(2)-LidarFOV(1))/Alpha); %number of points in Lidar scan
LIDAR_RANGE = 10;                           %LiDAR range in meters

%--> Select the solution type
% 0 : Pure LiDAR odometry solution
% 1 : Inertially-aided LiDAR odometry solution
% 2 : Pure Inertial solution
solution_type = 0;

%--> Gyroscope Noise Parameters
rng(0);
gaussian_noise = 0.5*pi/180*randn(1,Num_of_epochs);
bias_error = 0.1*pi/180*ones(1,Num_of_epochs);
noise_vector = bias_error + gaussian_noise;

%--> Point Cloud Display to Show LiDAR SCANs
lidarPlayer = pcplayer([-5 10], [0 15], [0 10]);
xlabel(lidarPlayer.Axes, 'X (m)')
ylabel(lidarPlayer.Axes, 'Y (m)')
zlabel(lidarPlayer.Axes, 'Z (m)')
title(lidarPlayer.Axes, 'Raw Lidar Sensor Data')

%--> Loop to display raw LiDAR scan
skipFrames  = 1;
numFrames   = Num_of_epochs;
for n = 1 : skipFrames : numFrames
    %--> Read raw LiDAR scan as a local point cloud
    ptCloud = read_point_cloud(n,scan,Alpha, LidarFOV, L);
    %--> Display the point cloud
    view(lidarPlayer, ptCloud);
    pause(0.005);
end

%--> Create an empty view set
vSet = pcviewset;

%--> Create a a figure to display trajectory results
hFigBefore = figure('Name', 'View Set Display');
hAxBefore = axes(hFigBefore);

%--> NDT Algorithm Parameters
regGridSize       = 1;

%--> Initialize the first pose matrix from the ground-truth data
InitialPoseMatrix = [cos(A(1)) sin(A(1)) 0 x(n);-sin(A(1)) cos(A(1)) 0 y(1);0 0 1 0;0 0 0 1];
AbsolutePoseObject = rigidtform3d(InitialPoseMatrix);    

%--> A Pose Object for relative transformations
RelativePoseObject   = rigidtform3d;  

%--> Main processing loop

viewId = 1;
skipFrames  = 1;
numFrames   = Num_of_epochs;
displayRate = 20;

for n = 1 : skipFrames : numFrames

    %--> Read point cloud
    ptCloud = read_point_cloud(n,scan,Alpha, LidarFOV, L);
 
    if n == 1
        vSet = addView(vSet, viewId, AbsolutePoseObject, "PointCloud", ptCloud);
        viewId = viewId + 1;
        ptCloudPrev = ptCloud;
        continue;
    end

    %--> Inertial Odometry
    delta_t = t(n) - t(n-skipFrames);
    dA = wz(n)*delta_t*pi/180;
    dX = speed(n)*sin(dA)*delta_t;
    dY = speed(n)*cos(dA)*delta_t;

    dA = dA + noise_vector(n);

    RelativeInertialPoseMatrix = [cos(dA) sin(dA) 0 dX;-sin(dA) cos(dA) 0 dY;0 0 1 0;0 0 0 1];

    if solution_type == 0
        RelativePoseObject = pcregisterndt(ptCloud, ptCloudPrev, regGridSize);
    elseif solution_type == 1
        InertialRelativePoseObject = rigidtform3d(RelativeInertialPoseMatrix);
        RelativePoseObject = pcregisterndt(ptCloud, ptCloudPrev, regGridSize,"InitialTransform", InertialRelativePoseObject);
    else
        InertialRelativePoseObject = rigidtform3d(RelativeInertialPoseMatrix);
        RelativePoseObject = InertialRelativePoseObject;
    end

    AbsolutePoseObject = rigidtform3d( AbsolutePoseObject.A * RelativePoseObject.A );

    %--> Add current local point cloud to the view set
    vSet = addView(vSet, viewId, AbsolutePoseObject, "PointCloud", ptCloud);
    
    %--> Add a connection from the previous view to the current view
    vSet = addConnection(vSet, viewId-1, viewId, RelativePoseObject);
    
    viewId = viewId + 1;
        
    ptCloudPrev = ptCloud;
    
    if mod(n,displayRate) == 0
        hG = plot(vSet, "Parent", hAxBefore);
        drawnow update
        view(0,90);grid on;
    end
end

absPoses = vSet.Views.AbsolutePose;

for k = 1:Num_of_epochs
    poses_ref(k,1) = x(k);
    poses_ref(k,2) = y(k);
    poses_ref(k,3) = -A(k)*pi/180;
end

hold on;plot(poses_ref(:,1),poses_ref(:,2),"--",'color','k','LineWidth',2);
xlabel('X(m)');ylabel('Y(m)');
if solution_type == 0
    title('\rm Pure LiDAR Odometry Solution');
    legend('LiDAR Odometry','Ground Truth');
elseif solution_type == 1
    title('\rm Inertially-Aided LiDAR Odometry Solution');
    legend('Inertial-LiDAR Odometry','Ground Truth');
else
    title('\rm Pure Inertial Solution');
    legend('Inertial Solution','Ground Truth');
end