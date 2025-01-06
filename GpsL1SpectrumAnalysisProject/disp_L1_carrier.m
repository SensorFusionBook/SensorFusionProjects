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

load L1_signal.mat;

Fs = 38.192e6;
X = cosCarr;
L = length(X);

[pxx,f] = pspectrum(X,Fs,'TwoSided',true);
figure;

plot(f/1e6,pow2db(abs(pxx).^2 / L / Fs) + 30);
grid on;hold on;
xlabel('Frequency (MHz)')
ylabel('Power Spectrum (dBm)')
title('Power Spectrum of Down converted L1 Carrier Sample')

load GPS_CA_Codes.mat;
ca1 = caCodesTable(1,(1:end));

combined_signal = cosCarr.*ca1;

X = combined_signal;
L = length(X);

[pxx,f] = pspectrum(X,Fs,'TwoSided',true);
figure;

plot(f/1e6,pow2db(abs(pxx).^2 / L / Fs) + 30);
grid on;hold on;
xlabel('Frequency (MHz)')
ylabel('Power Spectrum (dBm)')
title('Power Spectrum of Combined Code and Carrier')
