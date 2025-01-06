%{
Gnss Positioning Estimation Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

%--> Settings
clear;
clc;
close all;
fclose all;
set(0,'DefaultFigureWindowStyle','docked');

%--> Parameters
Threshold1 = 3;
MaximumLstSqrIterations = 20;
x = [0 0 0 0];   %Initial guess of position (in ECEF) and receiver clock bias
speed_of_light = 299792458;%m/s; speed of light
f0 = 10.23E6;       % GPS center frequency
f1 = 154*f0;		% L1 frequency Hz

wgs84 = referenceEllipsoid('wgs84');

%--> Load data file
load('ems_data.mat');

lat0 = ems_data.gnss_receiver_lat(1)*180/pi; % reference lat (for North-East-Up calculations)
lon0 = ems_data.gnss_receiver_lon(1)*180/pi; % reference lon (for North-East-Up calculations)
h0 = ems_data.gnss_receiver_up(1); % reference height (for North-East-Up calculations)

%--> Initialize some data structures
num_of_visible_satellites_record = zeros(1,length(ems_data.sat_time));
PDOP = zeros(1,length(ems_data.sat_time));

%--> Main Loop
for epoch_idx = 1:length(ems_data.sat_time)
    LstSqrIterationCounter = 0;
    pseudorange_stdv = ems_data.sat_range_std{:,epoch_idx}*25; %scaled standard deviation of pseudo-ranges
    R_matrix = diag(pseudorange_stdv.^2); % measurement covariance matrix
    pseudoranges = ems_data.sat_range{:,epoch_idx} - ems_data.iono_corrections{:,epoch_idx} - ems_data.tropo_clock_corrections{:,epoch_idx} + ems_data.sat_clock_corrections{:,epoch_idx};
    DeltaX = [10 10 10 10]; % Initial DeltaX
    num_of_visible_satellites = length(pseudoranges);
    num_of_visible_satellites_record(epoch_idx) = num_of_visible_satellites;

    %--> Begin Least-Sqaure Position Estimation Loop
    while ( (((sum(DeltaX.^2)) > Threshold1)))
        % Calculate H matrix
        H = BuildDesignMatrix(num_of_visible_satellites ,  ems_data.sat_Px{:,epoch_idx} ,ems_data.sat_Py{:,epoch_idx} ,ems_data.sat_Pz{:,epoch_idx} , x );      
        % Calcuate Residuals (difference between predicted measurement and observed measurements)
        Delta_PseudoRange = CalculatePseudoRangeError( 1 ,num_of_visible_satellites , pseudoranges,  ems_data.sat_Px{:,epoch_idx} ,ems_data.sat_Py{:,epoch_idx} ,ems_data.sat_Pz{:,epoch_idx} , x );
        % Calculate state estimation error dx
        DeltaX = ((H'/R_matrix)* H)\((H'/R_matrix)*Delta_PseudoRange);
        % Update/Correct the initial estimated x
        x = x + DeltaX(1:4)';
 
        LstSqrIterationCounter = LstSqrIterationCounter + 1;
        
        % Keep some variables for plots purposes
        res(epoch_idx,LstSqrIterationCounter,:) = abs(Delta_PseudoRange);
        dX(epoch_idx,LstSqrIterationCounter,:) = DeltaX;

        if (LstSqrIterationCounter > MaximumLstSqrIterations)
            break;
        end
    end

    if epoch_idx == 1
        figure;subplot(2,1,1);
        plot((1:LstSqrIterationCounter),dX(:,:,1),'color','r','linewidth',2);hold on;grid on;
        curtick = get(gca, 'xTick'); xticks(unique(round(curtick)));
        plot((1:LstSqrIterationCounter),dX(:,:,2),'color','g','linewidth',2);
        plot((1:LstSqrIterationCounter),dX(:,:,3),'color','b','linewidth',2);
        title('GPS SPP Algorthm: Convergence of Position Errors (Initial XYZ = {0.0, 0.0, 0.0})');
        legend('X','Y','Z');
        xlabel('Iteration');ylabel('Estimated dx(m)');
        subplot(2,1,2);plot((1:LstSqrIterationCounter),res(:,:,1),'r','linewidth',2);hold on;grid on;
        curtick = get(gca, 'xTick'); xticks(unique(round(curtick)));
        plot((1:LstSqrIterationCounter),res(:,:,2),'g','linewidth',2);plot((1:LstSqrIterationCounter),res(:,:,3),'b','linewidth',2);
        title('GPS SPP Algorthm: Absolute Range Residuals (m) (Initial XYZ = {0.0, 0.0, 0.0})');
        legend('r1','r2','r3');
        xlabel('Iteration');ylabel('Absolute Range Residuals (m)');
    end

    clear res;
    clear dX;

    DOP_ECEF = inv(H'*H);
    PDOP(epoch_idx) = sqrt(DOP_ECEF(1,1)^2+DOP_ECEF(2,2)^2+DOP_ECEF(3,3)^2);

    est_gnss_receiver_X(epoch_idx) = x(1);
    est_gnss_receiver_Y(epoch_idx) = x(2);
    est_gnss_receiver_Z(epoch_idx) = x(3);

    lat_long_alt = ecef2lla([x(1) x(2) x(3)],'WGS84');

    est_lat(epoch_idx) = lat_long_alt(1);
    est_lon(epoch_idx) = lat_long_alt(2);
    est_alt(epoch_idx) = lat_long_alt(3);

    [EastDOP(epoch_idx),NorthDOP(epoch_idx),UpDOP(epoch_idx)] = ecef2enu(DOP_ECEF(1,1)+x(1),DOP_ECEF(2,2)+x(2),DOP_ECEF(3,3)+x(3),est_lat(epoch_idx),est_lon(epoch_idx),est_alt(epoch_idx),wgs84);
    
    [est_gnss_receiver_east(epoch_idx),est_gnss_receiver_north(epoch_idx),est_gnss_receiver_up(epoch_idx)] = ecef2enu(x(1),x(2),x(3),lat0,lon0,0.0,wgs84);
    
    receiver_clock_bias(epoch_idx) = x(4);
   
    [xEast(epoch_idx),yNorth(epoch_idx),zUp(epoch_idx)] = ecef2enu(x(1),x(2),x(3),lat0,lon0,h0,wgs84);

    disp(strcat('Epoch : ',num2str(epoch_idx),' n_sv = ',num2str(num_of_visible_satellites),' iteration_Cnt = ',num2str(LstSqrIterationCounter)));
    
end

figure;
subplot(3,1,1);
plot(ems_data.sat_time , abs(EastDOP),'-*','Color','b');grid on;
title('GPS SPP Algorthm: EDOP');xlabel('time(s)');ylabel('EDOP');ylim([-0.5 2.5]);
subplot(3,1,2);
plot(ems_data.sat_time , abs(NorthDOP),'-*','Color','b');grid on;
title('GPS SPP Algorthm: NDOP');xlabel('time(s)');ylabel('NDOP');ylim([-0.5 2.5]);
subplot(3,1,3);
plot(ems_data.sat_time , abs(UpDOP),'-*','Color','b');grid on;
title('GPS SPP Algorthm: VDOP');xlabel('time(s)');ylabel('VDOP');ylim([-0.5 2.5]);

figure;
subplot(3,1,1);
plot(ems_data.sat_time , sqrt(EastDOP.^2 + NorthDOP.^2 + UpDOP.^2),'-*','Color','b');grid on;
title('GPS SPP Algorthm: PDOP');xlabel('time(s)');ylabel('PDOP');ylim([-0.5 2.5]);

subplot(3,1,2);
plot(num_of_visible_satellites_record,'-*','Color','r');grid on;
title('GPS SPP Algorthm: Number of Visible Satellites');xlabel('time(s)');ylabel('#SVs');ylim([10 17]);
subplot(3,1,3);
plot(ems_data.sat_time,abs(est_gnss_receiver_east - ems_data.gnss_receiver_east'),'-O','color','r');grid on;hold on;
plot(ems_data.sat_time,abs(est_gnss_receiver_north - ems_data.gnss_receiver_north'),'-square','color','k');grid on;
plot(ems_data.sat_time,abs(est_gnss_receiver_up - ems_data.gnss_receiver_up'),'-diamond','color','b');grid on;
title('GPS SPP Algorthm: Absolute Position Error (m)');xlabel('time(s)');ylabel('Error(m)');
legend('East Error','North Error','Up Error');

figure;
subplot(3,1,1);
plot(ems_data.sat_time,est_gnss_receiver_east,'-*','color','r','LineWidth',0.5);hold on;grid on;
plot(ems_data.sat_time,ems_data.gnss_receiver_east,'-*','color','b','LineWidth',0.5);xlabel('time(s)');ylabel('East(m)');grid on;
legend('GPS SPP Algorthm','Ground Truth');title('GPS SPP Algorthm Results : East(m)');

subplot(3,1,2);
plot(ems_data.sat_time,est_gnss_receiver_north,'-*','color','r','LineWidth',0.5);hold on;
plot(ems_data.sat_time,ems_data.gnss_receiver_north,'-*','color','b','LineWidth',0.5);xlabel('time(s)');ylabel('North(m)');grid on;
legend('GPS SPP Algorthm','Ground Truth');title('GPS SPP Algorthm Results : North(m)');

subplot(3,1,3);
plot(ems_data.sat_time,est_gnss_receiver_up,'-*','color','r','LineWidth',1);hold on;
plot(ems_data.sat_time,ems_data.gnss_receiver_up,'-*','color','b','LineWidth',1);xlabel('time(s)');ylabel('Up(m)');grid on;
legend('GPS SPP Algorthm','Ground Truth');title('GPS SPP Algorthm Results : Height(m)');ylim([0 100]);
