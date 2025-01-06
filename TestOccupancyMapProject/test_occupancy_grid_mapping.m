%{
Occupancy map Illustration Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

set(0,'DefaultFigureWindowStyle','docked');
clear;clc;close all;

%--> LiDAR sensor specifications
Alpha = 1.5;                                %angular resolution
LidarFOV = [-20 200];                       %Lidar Field of View

%--> Load data
load 'simulated_lidar_data.mat';
%{
    The file "simulated_lidar_data.mat" has the folowing data:
    
    A       : robot heading angle
    x,y     : robot 2D position
    wz      : robot angular velocity (turn rate)
    speed   : robot forward speed
    scan    : 2D array of the LiDAR range scans

%}

%--> Map Dimensions and Boundaries
[~,L] = size(scan);
N = length(t);
H = 9;
W = 11;
GRID_RESOLUTION = 10;
map = occupancyMap(W,H,GRID_RESOLUTION);
MAP = 0.5*ones(map.GridSize);
H = map.GridSize(1);
W = map.GridSize(2);
X_offset = 5.5*GRID_RESOLUTION;
Y_offset = 8.0*GRID_RESOLUTION;

%--> Main Processing Loop
for i = 1:N

    %--> Build the LiDAR scan
    for k = 1:L
        beam_angle = LidarFOV(1) + (k-1)*Alpha;
        if isnan(scan(i,k))
            scan(i,k) = 0;
        end
        ranges(k) = scan(i,k);
        angles(k) = beam_angle;
        beam_range = scan(i,k);        
        sensor_angle = beam_angle - A(i);
    end
    %--> Add poses and LiDAR scan objects to arrays
    scanobj = lidarScan(ranges,angles*pi/180);
    scans{i} = scanobj;
    poses(i,1) = x(i);
    poses(i,2) = y(i);
    poses(i,3) = -A(i)*pi/180;

    fprintf('Epoch %d/%d scan added\n',i,N);
end

%--> Process the arrays of poses and scans to build an occupancy map
map = buildMap(scans,poses,GRID_RESOLUTION,10);

%--> Display the map
show(map);
