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
subplot(3,3,1);
plot(time_vector_sec(1:NumRecords),ems_data.east(1:NumRecords),'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),EP_value(1:NumRecords),'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('east position (m)');xlabel('time(s)');
subplot(3,3,2);
plot(time_vector_sec(1:NumRecords),ems_data.north(1:NumRecords),'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),NP_value(1:NumRecords),'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('north position (m)');xlabel('time(s)');
subplot(3,3,3);
plot(time_vector_sec(1:NumRecords),ems_data.h(1:NumRecords),'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),alt_value(1:NumRecords),'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('up position (m)');xlabel('time(s)');

subplot(3,3,4);
plot(time_vector_sec(1:NumRecords),ems_data.vel_N(1:NumRecords,1),'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),ve_value(1:NumRecords),'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('east velocity (m/s)');xlabel('time(s)');
subplot(3,3,5);
plot(time_vector_sec(1:NumRecords),ems_data.vel_N(1:NumRecords,2),'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),vn_value(1:NumRecords),'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('north velocity (m/s)');xlabel('time(s)');
subplot(3,3,6);
plot(time_vector_sec(1:NumRecords),ems_data.vel_N(1:NumRecords,3),'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),vu_value(1:NumRecords),'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('up velocity (m/s)');xlabel('time(s)');

subplot(3,3,7);
plot(time_vector_sec(1:NumRecords),ems_data.roll(1:NumRecords)*R2D,'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),Euler_roll_value(1:NumRecords)*R2D,'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('roll (deg)');xlabel('time(s)');
subplot(3,3,8);
plot(time_vector_sec(1:NumRecords),ems_data.pitch(1:NumRecords)*R2D,'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),Euler_pitch_value(1:NumRecords)*R2D,'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('picth (deg)');xlabel('time(s)');
subplot(3,3,9);
plot(time_vector_sec(1:NumRecords),ems_data.heading(1:NumRecords)*R2D,'b','LineWidth',2);hold on;
plot(time_vector_sec(1:NumRecords),Euler_heading_value(1:NumRecords)*R2D,'r','LineWidth',2);grid on;
legend('ground-truth','processed');
title('heading (deg)');xlabel('time(s)');
