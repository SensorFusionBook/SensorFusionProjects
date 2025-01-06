%{
Gps L1 Spectrum Analysis Project

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

fs = 38.192e6;
D = 1;
fs = fs/D;

load GPS_CA_Codes.mat

delay_sec = 2*10^-4;
delay_samples = round(delay_sec*fs);

ca1 = caCodesTable(1,(1:D:end));

ca1_delayed = circshift(ca1,delay_samples);

[xcf,lags] = crosscorr(ca1,ca1_delayed,'NumLags',round(length(ca1)/2));

figure;plot(lags*(1/fs)*1000,xcf,'LineWidth',2);grid on;xlabel('delay (ms)');ylabel('correlation');
ylim([-0.2 1.0]);

X = ca1;
L = length(X);
Fs = fs;

[pxx,f] = pspectrum(X,'TwoSided',true);
figure;

plot(f,pow2db(abs(pxx).^2 / L / Fs) + 30);
grid on;hold on;
plot(f,randn(1,length(f))-115,'r');
legend('C/A Code','Noise Floor');

xlabel('Frequency (Hz)')
ylabel('Power Spectrum (dBm)')
title('Power Spectrum of C/A PRN1 Code')

return;

% figure;
% Y = fft(X);
% P2 = mag2db(abs(Y/L)/);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = (Fs/L)*(0:(L/2));
% plot(f,P1,'linewidth',2) 
% title("Single-Sided Amplitude Spectrum of C/A code PRN1")
% xlabel("f (Hz)")
% ylabel("|dB|");
% grid on;