%{
Matched Filter Demo Project

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

load GPS_CA_Codes.mat

ca1 = caCodesTable(1,(1:1000));

ca1_reversed = flipud(ca1);

ca1_attenuated_noisy = 0.6*ca1 + 0.1*randn(1,length(ca1));

ca1_attenuated_noisy_delayed = circshift(ca1_attenuated_noisy,250);

[xcf,lags] = crosscorr(ca1,ca1_attenuated_noisy_delayed,'NumLags',999);

figure;
subplot(3,1,1);plot(ca1,'linewidth',2,'Color','r');grid on;ylim([-1.2 1.2]);
title('Transmitted Code Pulse');xlabel('samples');
subplot(3,1,2);plot(ca1_attenuated_noisy_delayed,'linewidth',2,'Color','b');grid on;
title('Simulated Noisy Attenuated Echo Delayed by 250 Samples');xlabel('samples');
subplot(3,1,3);plot(lags,xcf,'linewidth',2,'Color','b');grid on;
title('Cross Correlation Output');xlabel('Delay (samples)');
