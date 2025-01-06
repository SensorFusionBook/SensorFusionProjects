%{
Inertial Navigation System Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

%}

%--> Plot error curves
figure;
subplot(3,3,1);
plot(time_vector_sec(1:NumRecords),E_error(1:NumRecords),'r','LineWidth',2);grid on;
title('east position error (m)');xlabel('time(s)');ylabel('');
subplot(3,3,2);
plot(time_vector_sec(1:NumRecords),N_error(1:NumRecords),'b','LineWidth',2);grid on;
title('north position error (m)');xlabel('time(s)');
subplot(3,3,3);
plot(time_vector_sec(1:NumRecords),U_error(1:NumRecords),'b','LineWidth',2);grid on;
title('altitude position error (m)');xlabel('time(s)');

subplot(3,3,4);
plot(time_vector_sec(1:NumRecords),ve_error(1:NumRecords),'r','LineWidth',2);grid on;
title('east velocity error (m/s)');xlabel('time(s)');
subplot(3,3,5);
plot(time_vector_sec(1:NumRecords),vn_error(1:NumRecords),'b','LineWidth',2);grid on;
title('north velocity error (m/s)');xlabel('time(s)');
subplot(3,3,6);
plot(time_vector_sec(1:NumRecords),vu_error(1:NumRecords),'b','LineWidth',2);grid on;
title('up velocity error (m/s)');xlabel('time(s)');

subplot(3,3,7);
plot(time_vector_sec(1:NumRecords),roll_error(1:NumRecords),'r','LineWidth',2);grid on;
title('roll error (deg)');xlabel('time(s)');
subplot(3,3,8);
plot(time_vector_sec(1:NumRecords),pitch_error(1:NumRecords),'b','LineWidth',2);grid on;
title('pitch error (deg)');xlabel('time(s)');
subplot(3,3,9);
plot(time_vector_sec(1:NumRecords),heading_error(1:NumRecords),'b','LineWidth',2);grid on;
title('heading error (deg)');xlabel('time(s)');
