%{
Gauss - Markov process Illustration Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

set(0,'DefaultFigureWindowStyle','docked');
clear;clc;close all;

Fs = 20;        % Sampling frequency                    
dT = 1/Fs;      % Sampling period    
T = 60;         % Duration
L = T/dT;       % Length of signal
t = 0:dT:T;     % Time vector
time_const = 5;
stdv = 1.0;

gm(1) = 0.0;

for i = 1:length(t)-1
    gm(i+1) = gm(i) + ((-1/time_const)*gm(i) + stdv*randn(1,1))*dT;
end

figure;
plot(t,gm,'linewidth',2);grid on;title('\rm Gaussian Markov process sample');xlabel('t (s)');
ylim([-1 1]);

[acf,lags] = autocorr(gm,NumLags=L);
figure;
plot(lags*dT,acf,'linewidth',2);grid on;title('\rm Autocorrelation of Gaussian Markov process sample');xlabel('\tau (s)');

X = gm;
N = length(X);
xdft = fft(X);
xdft = xdft(1:floor(N/2+1));
psdx = (1/(Fs*N)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:Fs/length(X):Fs/2;

y = pow2db(psdx);

ys = smoothdata(y,"gaussian",60);

figure;
plot(freq,y,'LineWidth',2);
hold on;plot(freq,ys,"Color",'red','LineWidth',2);
legend('PSD','Smoothed PSD');grid on
title("\rm Power spectral density (PSD) of Gaussian Markov process sample")
xlabel("Frequency (Hz)")
ylabel("Power/Frequency (dB/Hz)")