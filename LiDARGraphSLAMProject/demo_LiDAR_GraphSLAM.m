%{
LiDAR Graph SLAM Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

set(0,'DefaultFigureWindowStyle','docked');
clear;clc;close all;

%--> Load data

load ('simulated_lidar_data.mat');

%--> LiDAR sensor specs
Alpha = 1.5;                                %angular resolution in degrees
LidarFOV = [-20 200];                       %Lidar Field of View in degrees
L = round((LidarFOV(2)-LidarFOV(1))/Alpha); %number of points in Lidar scan
LIDAR_RANGE = 10;                           %LiDAR range in meters
N = length(t);
Ts = 0.2;
apply_automatic_loop_detector = 0;

%--> Create a streaming point cloud display player object
lidarPlayer = pcplayer([-5 10], [0 15], [0 10]);
xlabel(lidarPlayer.Axes, 'X (m)')
ylabel(lidarPlayer.Axes, 'Y (m)')
zlabel(lidarPlayer.Axes, 'Z (m)')
title(lidarPlayer.Axes, 'Lidar Sensor Data')

%--> Display lidar sensor data

skipFrames  = 1;
numFrames   = N;
lidarscans = cell(numFrames,1);
for n = 1 : skipFrames : numFrames
    ptCloud = read_point_cloud(n,scan,Alpha, LidarFOV, L);
    view(lidarPlayer, ptCloud);    
    scanob = read_lidar_scan(n,scan,Alpha, LidarFOV, L);
    lidarscans{n} = scanob;
    pause(0.005);
end

%--> Run SLAM
% Initailization
vSet = pcviewset;
loopDetector = scanContextLoopDetector;
maxTolerableRMSE  = .4;
hFigBefore = figure('Name', 'View Set Display');
hAxBefore = axes(hFigBefore);
regGridSize       = 1;
noise_vector = 0.1*pi/180*ones(1,N);
T = [cos(A(1)) sin(A(1)) 0 x(n);
    -sin(A(1)) cos(A(1)) 0 y(1);
    0 0 1 0;0 0 0 1];
absTform = rigidtform3d(T);    
relTform   = rigidtform3d;  
viewId = 1;
skipFrames  = 1;
numFrames   = N;
displayRate = 20;

% Main processing loop
for n = 1 : skipFrames : numFrames
    ptCloud = read_point_cloud(n,scan,Alpha, LidarFOV, L);
    firstFrame = (n==1);
    if firstFrame
        vSet = addView(vSet, viewId, absTform, "PointCloud", ptCloud);
        viewId = viewId + 1;
        ptCloudPrev = ptCloud;
        continue;
    end

    % Inertial odometry
    delta_t = t(n) - t(n-skipFrames);
    dA = wz(n)*delta_t*pi/180;
    dA = dA + noise_vector(n);
    dX = speed(n)*sin(dA)*delta_t;
    dY = speed(n)*cos(dA)*delta_t;
    T = [cos(dA) sin(dA) 0 dX;
        -sin(dA) cos(dA) 0 dY;
        0 0 1 0;
        0 0 0 1];
    initTform = rigidtform3d(T);
    
    relTform = initTform;

    absTform = rigidtform3d( absTform.A * relTform.A );

    vSet = addView(vSet, viewId, absTform, "PointCloud", ptCloud);
    
    vSet = addConnection(vSet, viewId-1, viewId, relTform);
    
    if apply_automatic_loop_detector
        descriptor = scanContextDescriptor(ptCloud);
        addDescriptor(loopDetector, viewId, descriptor)
        loopViewId = detectLoop(loopDetector);
        if ~isempty(loopViewId)
            loopViewId = loopViewId(1);
            loopView = findView(vSet, loopViewId);
            ptCloudOld = loopView.PointCloud;
            [relTform, ~, rmse] = pcregisterndt(ptCloud, ptCloudOld, ...
                regGridSize, "MaxIterations", 50);
    
            acceptLoopClosure = rmse <= 0.09;
            if acceptLoopClosure
                infoMat = 0.01 * eye(6);
                vSet = addConnection(vSet, loopViewId, viewId, relTform, infoMat);
            end
        end
    else
        if viewId == N
            loopViewId = 1;
            dA = (A(viewId)-A(loopViewId))*pi/180;
            dX = x(viewId) - x(loopViewId);
            dY = y(viewId) - y(loopViewId);
            dT = (absTform.R')*[dX;dY;0];
            dX = dT(1);dY = dT(2);
            T = [cos(dA) sin(dA) 0 dX;-sin(dA) cos(dA) 0 dY;0 0 1 0;0 0 0 1];
            relTform = rigidtform3d(T);
            infoMat = 0.01 * eye(6);
            vSet = addConnection(vSet, loopViewId, viewId, relTform, infoMat);
            hG = plot(vSet, "Parent", hAxBefore);
            drawnow update;
            view(0,90);grid on;
        end
    end

    viewId = viewId + 1;
        
    ptCloudPrev = ptCloud;
    initTform   = relTform;
    
    if mod(n,displayRate) == 0
        hG = plot(vSet, "Parent", hAxBefore);
        drawnow update
        view(0,90);grid on;
    end
end

G = createPoseGraph(vSet);
loopEdgeIds = find(abs(diff(G.Edges.EndNodes, 1, 2)) > 1);
highlight(hG, 'Edges', loopEdgeIds, 'EdgeColor', 'red', 'LineWidth', 3)

optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
vSetOptim = updateView(vSet, optimG.Nodes);

GRID_RESOLUTION = 10;
absPoses = vSet.Views.AbsolutePose;
absPosesOptim = vSetOptim.Views.AbsolutePose;

%--> Calculate reference solution and graph-optimized solution
poses_ref = zeros(N,3);
poses = zeros(N,3);
posesoptim = zeros(N,3);
for k = 1:N
    poses_ref(k,1) = x(k);
    poses_ref(k,2) = y(k);
    poses_ref(k,3) = -A(k)*pi/180;

    T = [cos(A(k)) sin(A(k)) 0 x(k);
        -sin(A(k)) cos(A(k)) 0 y(k);
        0 0 1 0;
        0 0 0 1];
    
    [~,~,heading] = dcm2angle(absPoses(k).R,'XYZ');
    poses(k,1) = absPoses(k).Translation(1);
    poses(k,2) = absPoses(k).Translation(2);
    poses(k,3) = -heading; 

    [~,~,heading] = dcm2angle(absPosesOptim(k).R,'XYZ');
    posesoptim(k,1) = absPosesOptim(k).Translation(1);
    posesoptim(k,2) = absPosesOptim(k).Translation(2);
    posesoptim(k,3) = -heading; 
end

hold on;plot(poses_ref(:,1),poses_ref(:,2),"--",'color','k','LineWidth',2);
xlabel('X(m)');ylabel('Y(m)');
title('\rm LiDAR Graph-SLAM before applying loop closure');
legend('\rm LiDAR Graph-SLAM','Ground truth');

%--> Plot 2-D trajectory results
figure;plot(poses_ref(:,1),poses_ref(:,2),'--','color','b','LineWidth',2);hold on;
plot(poses(:,1),poses(:,2),'-.','color','r','LineWidth',2);grid on;
plot(posesoptim(:,1),posesoptim(:,2),'-','color','g','LineWidth',2);
legend('Ground truth','SLAM without loop closure','SLAM with loop closure');
title('\rm LiDAR graph-SLAM. Loop closure effect on the optimization');
xlabel('X(m)');ylabel('Y(m)');axis equal;
ylim([-1 10]);

figure;map = buildMap(lidarscans,poses_ref,GRID_RESOLUTION,10);show(map);title('\rm Ground truth occupancy map');
figure;map = buildMap(lidarscans,poses,GRID_RESOLUTION,10);show(map);title('\rm  Occupancy map without loop closure');
figure;map = buildMap(lidarscans,posesoptim,GRID_RESOLUTION,10);show(map);title('\rm Occupancy map with loop closure');