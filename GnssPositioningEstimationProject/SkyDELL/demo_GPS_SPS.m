%{
Gnss Positioning Estimation Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

set(0,'DefaultFigureWindowStyle','docked');
clear;close all;fclose all;clc;

%--> Load data file
load gps_measurements.mat;

%--> Define constants
c   = 299792458;            % Speed of light
we  = 7.2921151476e-5;      % Earth rotation rate in rad/sec

ReferenceRxPos_ECEF = [receiver_measurements.ECEFX_m_,receiver_measurements.ECEFY_m_,receiver_measurements.ECEFZ_m_];
ReferenceRxPos_GEOD = ecef2lla(ReferenceRxPos_ECEF);

%--> Plot the 2-D trajectory on a map for display
figure;
geoplot(ReferenceRxPos_GEOD(:,1),ReferenceRxPos_GEOD(:,2),'*b');
hold on;
geobasemap("topographic");
title("Object Trajectory");

%--> Main processing loop
N = size(receiver_measurements,1);
PositionError_sagnac_corrected = zeros(N,3);
PositionError_sagnac_uncorrected = zeros(N,3);
EstimatedRxPos_ECEF_uncorrected = zeros(N,3);
EstimatedRxPos_ECEF_corrected = zeros(N,3);

for k = 1:N
    % Find visible satellites at current epoch
    visibleSat = [];
    for i = 1:length(sat_measurements)-3
        if any(receiver_measurements.ElapsedTime_ms_(k) == sat_measurements{i}.ElapsedTime_ms_)
            visibleSat(end+1) = i;
        end
    end

    NumOfVisibleSatellites = length(visibleSat);
    SatellitesPos_ECEF = zeros(3,NumOfVisibleSatellites);
    Pseudoranges = zeros(NumOfVisibleSatellites,1);
    SatClckBiases = zeros(NumOfVisibleSatellites,1);
    TroposphericErrors = zeros(NumOfVisibleSatellites,1);
    IonosphericErrors = zeros(NumOfVisibleSatellites,1);
    SignalTransmissionTime = zeros(NumOfVisibleSatellites,1);
    for i = 1:length(visibleSat)
        index = find(receiver_measurements.ElapsedTime_ms_(k) == sat_measurements{visibleSat(i)}.ElapsedTime_ms_); % Finding the index of the current epoch in table
        SatellitesPos_ECEF(:,i) = [sat_measurements{visibleSat(i)}.ECEFX_m_(index); sat_measurements{visibleSat(i)}.ECEFY_m_(index); sat_measurements{visibleSat(i)}.ECEFZ_m_(index)]; % Finding GPS satellite positions in ECEF
        Pseudoranges(i,1) = sat_measurements{visibleSat(i)}.PSR_m_(index); % Finding pseudorange
        SatClckBiases(i,1) = sat_measurements{visibleSat(i)}.ClockCorrection_s_(index);  % Finding satellite clock bias
        TroposphericErrors(i,1) = sat_measurements{visibleSat(i)}.TropoCorrection_m_(index); % Finding the tropospheric delay
        IonosphericErrors(i,1) = sat_measurements{visibleSat(i)}.IonoCorrection_m_(index);  % Finding the ionospheric delay
        SignalTransmissionTime(i,1) = sat_measurements{visibleSat(i)}.PSRSatelliteTime_ms_(index);  % Finding the signal transmission time
    end
    TimeOfArrivals = receiver_measurements.ElapsedTime_ms_(k) - SignalTransmissionTime;

    deltaW = we * TimeOfArrivals * 10^-3; % Earth Rotation change during the signal propagation

    %--> Correct for the Sagnac effect
    SatellitesPos_ECI = SatellitesPos_ECEF;
    for i = 1:length(visibleSat)
        M_ROT = [cos(deltaW(i)), sin(deltaW(i)), 0;
            -sin(deltaW(i)), cos(deltaW(i)), 0;
            0, 0, 1];
        SatellitesPos_ECI(:,i) = M_ROT*SatellitesPos_ECEF(:,i);
    end

    %--> Calculate the Receiver Position using Least-square Algorithm
    EstimatedRxPos_ECEF_uncorrected(k,:) = calculate_receiver_position(SatellitesPos_ECEF,Pseudoranges,c * SatClckBiases,IonosphericErrors,TroposphericErrors);
    PositionError_sagnac_uncorrected(k,:) = [receiver_measurements.ECEFX_m_(k),receiver_measurements.ECEFY_m_(k),receiver_measurements.ECEFZ_m_(k)] - EstimatedRxPos_ECEF_uncorrected(k,:);

    EstimatedRxPos_ECEF_corrected(k,:) = calculate_receiver_position(SatellitesPos_ECI,Pseudoranges,c * SatClckBiases,IonosphericErrors,TroposphericErrors);
    PositionError_sagnac_corrected(k,:) = [receiver_measurements.ECEFX_m_(k),receiver_measurements.ECEFY_m_(k),receiver_measurements.ECEFZ_m_(k)] - EstimatedRxPos_ECEF_corrected(k,:);

    fprintf('Epoch (%d) Processed\n',k);

end

EstimatedRxPos_GEOD_corrected = ecef2lla(EstimatedRxPos_ECEF_corrected);
EstimatedRxPos_GEOD_uncorrected = ecef2lla(EstimatedRxPos_ECEF_uncorrected);
geoplot(EstimatedRxPos_GEOD_corrected(:,1),EstimatedRxPos_GEOD_corrected(:,2),'*r');
geoplot(EstimatedRxPos_GEOD_uncorrected(:,1),EstimatedRxPos_GEOD_uncorrected(:,2),'*k');

legend('Ground truth trajectory','Estimated trajectory');


figure;
subplot(3,2,1);
t = linspace(1,N,N);
subplot(3,2,1);
plot(t,abs(PositionError_sagnac_corrected(:,1)),'*b');hold on;
plot(t,abs(PositionError_sagnac_uncorrected(:,1)),'*r');
legend('corrected','uncorrected');
xlabel('Snapshot');
ylabel('x bias(m)');
grid on;

subplot(3,2,3);
plot(t,abs(PositionError_sagnac_corrected(:,2)),'*b');hold on;
plot(t,abs(PositionError_sagnac_uncorrected(:,2)),'*r');
legend('corrected','uncorrected');
xlabel('Snapshot');
ylabel('y bias(m)');
grid on;

subplot(3,2,5);
plot(t,abs(PositionError_sagnac_corrected(:,3)),'*b');hold on;
plot(t,abs(PositionError_sagnac_uncorrected(:,3)),'*r');
legend('corrected','uncorrected');
xlabel('Snapshot');
ylabel('z bias(m)');
grid on;

subplot(3,2,[2,4,6]);
plot(t,vecnorm(PositionError_sagnac_corrected,2,2),'*b');hold on;
plot(t,vecnorm(PositionError_sagnac_uncorrected,2,2),'*r');
legend('corrected','uncorrected');

xlabel('Snapshot');
ylabel('3D positional accuracy(m)');
grid on;