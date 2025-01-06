%{
RADAR DSP Illustration Project

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

%--> RADAR sensor Parameters 
speed_of_light = 3e8;
carrier_freq = 77e9;
carrier_wv_len = (speed_of_light/carrier_freq);
BandWidth = 15000000; %(Hz)
ChirpTime = 1e-5; %(sec)
ChirpsPerSeconds = 1/ChirpTime;
FrequencySlope = BandWidth/ChirpTime; %slope
Samples_per_chirp = 1024;% number of samples per chirp
Chirps_per_frame = 128; % number of chirps per frame
SamplingFreq = Samples_per_chirp/ChirpTime;
maximum_range = speed_of_light*SamplingFreq*ChirpTime/(4*BandWidth);
maximum_velocity = carrier_wv_len/(4*ChirpTime);

%--> Targets Parameters (Simulate Three Targets)
target_1_range = 1000;
target_1_speed = 50;

target_2_range = 2000;
target_2_speed = -50;

target_3_range = 3000;
target_3_speed = 0;

%--> Timestamp vector
SimulationTimeVector = linspace(0,Chirps_per_frame*ChirpTime,Samples_per_chirp*Chirps_per_frame);

%--> Allocate vectors for Tx, Rx and the Mix signals.
TxSignal = zeros(1,length(SimulationTimeVector)); %transmitted signal
RxSignal = zeros(1,length(SimulationTimeVector)); %received signal
MixerOutputSignal = zeros(1,length(SimulationTimeVector)); %beat signal

%--> Main Simulation Loop

for i=1:length(SimulationTimeVector)
  %--> Generate the transmitted signal
  TxSignal(i)   = cos(2*pi*(carrier_freq*SimulationTimeVector(i) + (FrequencySlope*SimulationTimeVector(i)^2)/2 ) );

  %--> Simulate a reflection signal from target 1
  updated_target_range = target_1_range + (target_1_speed*SimulationTimeVector(i));
  updated_target_roundtrip_delay = (2 * updated_target_range) / speed_of_light;
  RxSignal(i)   = RxSignal(i) + cos(2*pi*(carrier_freq*(SimulationTimeVector(i) -updated_target_roundtrip_delay) + (FrequencySlope * (SimulationTimeVector(i)-updated_target_roundtrip_delay)^2)/2 ) );  

  %--> Simulate a reflection signal from target 2
  updated_target_range = (target_2_range) + ((target_2_speed)*SimulationTimeVector(i));
  updated_target_roundtrip_delay = (2 * updated_target_range) / speed_of_light;
  RxSignal(i) = RxSignal(i) + cos(2*pi*(carrier_freq*(SimulationTimeVector(i) -updated_target_roundtrip_delay) + (FrequencySlope * (SimulationTimeVector(i)-updated_target_roundtrip_delay)^2)/2 ) );

  %--> Simulate a reflection signal from target 3
  updated_target_range = (target_3_range) + ((target_3_speed)*SimulationTimeVector(i));
  updated_target_roundtrip_delay = (2 * updated_target_range) / speed_of_light;
  RxSignal(i) = RxSignal(i) + cos(2*pi*(carrier_freq*(SimulationTimeVector(i) - updated_target_roundtrip_delay) + (FrequencySlope * (SimulationTimeVector(i) - updated_target_roundtrip_delay)^2)/2 ) );

  %--> Simulate the mixed signal inside the RADAR receiver
  MixerOutputSignal(i) = TxSignal(i) .* RxSignal(i);
end

%--> Process the MixerOutputSignal to detect targets' ranges and their speeds

%--> Build the data cube from the mixer output stream
% The data cube in single-antenna RADAR is a 2-D array
% The columns correspond to the number of chirps in the frame
% The rows correspond to the number of samples per chirp
% Each column is a chirp-length buffer filled with data from the mixer
MixerOutputSignal = reshape(MixerOutputSignal, [Samples_per_chirp, Chirps_per_frame]);

%--> Apply FFT to detect target ranges
signal_fft = fft(MixerOutputSignal, Samples_per_chirp);
shifted_signal_fft = fftshift(signal_fft);
signal_fft = abs(signal_fft);
signal_fft = signal_fft ./ max(signal_fft); % Normalize
shifted_signal_fft = abs(shifted_signal_fft);
shifted_signal_fft = shifted_signal_fft ./ max(shifted_signal_fft); % Normalize

%--> Display the FFT results for the first frame in the data cube
figure;
plot(signal_fft(:,1));
title({'Double Sided Range FFT for the first chirp in the data cube',...
    'X-axis has N values where N = Number of samples per chirp', ...
    'Each x-axis index m is corresponding to a frequency mFs/N', ...
    'Fs is the sampling frequency'});
ylabel('Amplitude (Normalized)');
xlabel('Index within samples of a chirp');grid on;

figure;
SamplingFreq = 1/(ChirpTime/Samples_per_chirp);
plot((SamplingFreq/Samples_per_chirp*(-Samples_per_chirp/2:Samples_per_chirp/2-1))/1000000,shifted_signal_fft(:,1));
title('\rm Shifted Double Sided Range FFT. X-axis is converted to frequencies in MHz');
ylabel('Amplitude (Normalized)');
xlabel('f(MHz)');grid on;

figure;
plot((SamplingFreq/Samples_per_chirp*(0:Samples_per_chirp/2-1))/1000000,signal_fft(1:Samples_per_chirp/2,1));
title('\rm Single Sided Range FFT. X-axis is converted to frequencies in MHz');
ylabel('Amplitude (Normalized)');
xlabel('f(MHz)');grid on;

figure;
plot(SamplingFreq/Samples_per_chirp*(0:Samples_per_chirp/2-1)*speed_of_light/(2*FrequencySlope),signal_fft(1:Samples_per_chirp/2,1));
title('\rm Single Sided FFT vs. Range. X-axis is converted to ranges');
ylabel('Amplitude (Normalized)');
xlabel('Range (m)');grid on;

%--> Detect the peaks in the FFT of the first frame in the data cube
%--> locs points to the index of rows where we have a peak in the column (the chirp-length buffer)
[pks,locs] = findpeaks(signal_fft(:,1),'MinPeakHeight',0.5);

%--> Apply FFT twice (2D FFT) to detect both ranges and speed (dopplers)
%--> 2D FFT can be done in one step. But for clarification, we will in two steps
MixerOutputSignal=reshape(MixerOutputSignal,[Samples_per_chirp,Chirps_per_frame]);

%--> Apply the first FFT on the columns of the data cube
%--> Each column is a chirp-length buffer filled with mixed signal output
signal_fft = fft(MixerOutputSignal, Samples_per_chirp);

%--> Apply the second FFT on the rows where we have a peak (a target)

for i = 1:length(locs)
    
    %--> Show the FFT phase angle of the row where we have a peak (a target)
    %--> This phase angle is related to the doppler (speed) of the target
    figure;
    plot(angle(signal_fft(locs(1),:)));grid on;
    title(sprintf('FFT Phase Angle of row number %d',locs(i)));
    
    %--> Apply the second FFT on the rows where we have a peak (a target)
    doppler_fft = fft(signal_fft(locs(i),:));
    doppler_fft_shifted = fftshift(doppler_fft);
    doppler_fft = abs(doppler_fft);
    doppler_fft_shifted = abs(doppler_fft_shifted);
    doppler_fft = doppler_fft ./ max(doppler_fft); % Normalize
    doppler_fft_shifted = doppler_fft_shifted ./ max(doppler_fft_shifted); % Normalize
    
    figure;
    plot(doppler_fft);grid on;
    title('\rm Double Sided Doppler FFT. X-axis is the column index in the row where we have a peak (a target)');
    ylabel('Amplitude (Normalized)');
    xlabel('column index in the row where we have a peak (a target)');
    
    figure;
    plot((ChirpsPerSeconds/Chirps_per_frame*(-Chirps_per_frame/2:Chirps_per_frame/2-1))/1000,doppler_fft_shifted);
    title('\rm Double Sided Shifted Doppler FFT. X-axis is frequencies (KHz)');
    ylabel('Amplitude (Normalized)');
    xlabel('f(KHz)');grid on;

end

%--> Apply 2D FFT at once on the entire data cube
MixerOutputSignal=reshape(MixerOutputSignal,[Samples_per_chirp,Chirps_per_frame]);
signal_fft2 = fft2(MixerOutputSignal,Samples_per_chirp,Chirps_per_frame);
%signal_fft2 = fft(fft(MixerOutputSignal).').';    %gives the same results

%--> Display the results in a single sided view
signal_fft2_shifted = fftshift(signal_fft2);

figure;
surf( linspace(-maximum_velocity,maximum_velocity,Chirps_per_frame), linspace(0,maximum_range,Samples_per_chirp/2), 10*log10(abs(signal_fft2_shifted(Samples_per_chirp/2+1:end,:))) );
title('\rm 2-D FFT Output');
xlabel('Radial Speed (m/s)');
ylabel('Target Range (m)');
zlabel('Amplitude (dB)');
