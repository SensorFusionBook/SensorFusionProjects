%{
Inertial Navigation System Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

%}

figure;
subplot(1,2,1);
plot(ems_data.east(1:NumRecords),ems_data.north(1:NumRecords),'LineWidth',2);
hold on;grid on;title('2-D Trajectory Plot');
plot(EP_value(1:NumRecords),NP_value(1:NumRecords),'LineWidth',2);
xlabel('East(m)');ylabel('North(m)');
legend('ground-truth','processed');
subplot(1,2,2);
plot3(ems_data.east(1:NumRecords),ems_data.north(1:NumRecords),ems_data.h(1:NumRecords),'LineWidth',2);
hold on;grid on;title('3-D Trajectory Plot');
plot3(EP_value(1:NumRecords),NP_value(1:NumRecords),alt_value(1:NumRecords),'LineWidth',2);
legend('ground-truth','processed');
xlabel('East(m)');ylabel('North(m)');zlabel('Up(m)');
view(-12,10);