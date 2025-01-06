%{
Random walk process Illustration Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

set(0,'DefaultFigureWindowStyle','docked');
clear;clc;close all;

Fs = 1;            % Sampling frequency                    
dT = 1/Fs;         % Sampling period    
T = 60*60;         % Duration
L = T/dT;          % Length of signal
t = 0:dT:T;        % Time vector
S = 500;           % Number of samples
n_std = 1;
n = n_std*randn(size(t));
rw = zeros(S,length(t));

for i = 1:S
    n = n_std*randn(size(t));
    rw(i,:) = cumsum(n)*dT;
end

counter = 1;
for s = 60:60:t(end)

    t_rw_std(counter) = t(s/dT);
    rw_std(counter) = std(rw(:,(s/dT)));

    counter = counter + 1;
end

figure;
subplot(4,1,1);plot(t,n,'linewidth',2);grid on;title('\rm Gaussian random noise');xlabel('time (s)');
subplot(4,1,2);plot(t,rw,'linewidth',2);grid on;title('\rm Random walk (integration of random noise)');xlabel('time (s)');
subplot(4,1,3);plot(t_rw_std,rw_std,'linewidth',2);grid on;title('\rm Standard Deviation (STDV) of Random walk vs. integration time');xlabel('time(s)');
subplot(4,1,4);plot(sqrt(t_rw_std),rw_std,'linewidth',2);grid on;title('\rm Standard Deviation (STDV) of random walk vs. Sqrt of Integration Time');xlabel('sqrt(time(s))');

X = n;
figure;
N = length(X);
xdft = fft(X);
xdft = xdft(1:floor(N/2+1));
psdx = (1/(Fs*N)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:Fs/length(X):Fs/2;

y = pow2db(psdx);

ys = smoothdata(y,"gaussian",60);

plot(freq,y,'LineWidth',2);
hold on;plot(freq,ys,"Color",'red','LineWidth',2);
legend('PSD','Smoothed PSD');grid on
title("\rm Power spectral density (PSD) of Gaussian random noise")
xlabel("Frequency (Hz)")
ylabel("Power/Frequency (dB/Hz)")