%{
Demonstration of digital filter
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

Acknowledgements
The signal data used in this project is based on the software package bound
in the book https://www.mathworks.com/academia/books/a-software-defined-gps-and-galileo-receiver-borre.html
%}

set(0,'DefaultFigureWindowStyle','docked');
clear;clc;close all;

load signal0DC.mat;
load L1_signal.mat;
samplesPerCode = 38192;
GPS_IF_signal = signal0DC(1:samplesPerCode);
Local_carrier_signal = cosCarr(1:samplesPerCode);
Fs = 38.192e6;        % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = samplesPerCode;   % Length of signal
t = (0:L-1)*T;        % Time vector

figure;
plot(1000*t,GPS_IF_signal);grid on;
xlabel("t (milliseconds)");
ylabel("X(t)",'FontAngle', 'italic');
title('\rm Raw intermediate frequency GPS signal');

figure;
plot(1000*t,Local_carrier_signal);grid on;
xlabel("t (milliseconds)");
ylabel("X(t)",'FontAngle', 'italic');
title('\rm Local carrier signal');

%--> Double-sided amplitude spectrum of raw intermediate frequency GPS signal
y = fft(GPS_IF_signal);
z = fftshift(y);
ly = length(y);
f = (-ly/2:ly/2-1)/ly*Fs;
figure;
plot(f/1000000,abs(z/ly),'linewidth',2);grid on;
title('\rm Double-sided amplitude spectrum of raw intermediate frequency GPS signal');
xlabel("Frequency (MHz)");
ylabel("Amplitude");

%--> Double-sided amplitude spectrum of local carrier signal
y = fft(Local_carrier_signal);
z = fftshift(y);
ly = length(y);
f = (-ly/2:ly/2-1)/ly*Fs;
figure;
plot(f/1000000,abs(z/ly),'linewidth',2);grid on;
title('\rm Double-sided amplitude spectrum of local carrier signal');
xlabel("Frequency (MHz)");
ylabel("Amplitude");

%--> Calculate the mixed signal
mixed_signal = Local_carrier_signal.*GPS_IF_signal;
figure;
plot(1000*t,mixed_signal);grid on;
xlabel("t (milliseconds)");
ylabel("X(t)");
title('\rm Mixed signal');

%--> Double-sided amplitude spectrum of mixed signal
y = fft(mixed_signal);
z = fftshift(y);
ly = length(y);
f = (-ly/2:ly/2-1)/ly*Fs;
figure;
plot(f/1000000,abs(z/ly),'linewidth',2);grid on;
title('\rm Double-sided amplitude spectrum of mixed signal');
xlabel("Frequency (MHz)");
ylabel("Amplitude");

%--> Lowpass filtering
b = [0.0192 0.0486 0.0651 0.0486 0.0192];
a = [1 -2.0557 2.093 -1.0676 0.2333];
[h,f] = freqz(b,a,512,Fs);
plot(f/1000000,10*log10(abs(h)));grid on;
xlabel('Frequency (MHz)')
ylabel('Magnitude (dB)')
title('\rm Frequency response of the digital filter');
[filtered_signal,zf] = filter(b,a,mixed_signal);

%--> Power spectrum of raw, mixed, and filtered GPS signals
[p,f] = pspectrum([GPS_IF_signal' mixed_signal' filtered_signal'],Fs);
y = 10*log10(abs(p));
y1 = y(:,1);
y2 = y(:,2);
y3 = y(:,3);
figure;
plot(f/1e6,y1,'LineWidth',10,'color','b');hold on;
plot(f/1e6,y2,'LineWidth',10,'color','r');
plot(f/1e6,y3,'LineWidth',2,'Color','k','LineStyle','--');grid on;
legend('Raw signal','Mixed signal','Filtered signal');
title('\rm Power spectrum of raw, mixed, and filtered GPS signals');
xlabel("Frequency (MHz)");
ylabel("Amplitude");

figure;
plot(1000*t,filtered_signal);grid on;
xlabel("t (milliseconds)");
ylabel("X(t)",'FontAngle', 'italic');
title('\rm Filtered signal');

figure;
y = fft(filtered_signal);
z = fftshift(y);
ly = length(y);
f = (-ly/2:ly/2-1)/ly*Fs;
plot(f/1000000,abs(z/ly),'linewidth',2);grid on;
title('\rm Double-sided amplitude spectrum of filtered signal');
xlabel("Frequency (MHz)");
ylabel("Amplitude");