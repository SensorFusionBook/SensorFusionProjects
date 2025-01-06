%{
Demonstration of Allan Variance
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

function plot_imu_data(t,Acc,Gyr,title_text)

    figure;
    subplot(2,1,1);
    plot(t,Acc(:,3),'r');grid on;
    legend('z');title(strcat(title_text,'Acc Z'));
    subplot(2,1,2);
    plot(t,Acc(:,1),'g');grid on;hold on;
    plot(t,Acc(:,2),'b');
    ylabel('Acceleration (m/s^2)')
    legend('x','y');grid on;
    xlabel('t(sec)');
    title(strcat(title_text,'Acc X,Y'));

    figure;plot(t,Gyr(:,1),'r');title(strcat(title_text,' Gyro'));hold on;
    plot(t,Gyr(:,2),'g');
    plot(t,Gyr(:,3),'b');
    ylabel('Gyroscope (deg/s)');
    legend('x','y','z');grid on;
    xlabel('t(sec)');

end