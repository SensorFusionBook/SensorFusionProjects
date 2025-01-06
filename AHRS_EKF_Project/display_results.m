%{
Attitude and Heading Reference System (AHRS) Project using EKF
Sensors include Magnetometer, Accelerometer, and Gyroscope
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

close all;

time_vector = time_vector_sec(1:end)-time_vector_sec (1);

%--> 
figure;
subplot(3,1,1);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_x*R2D, 'k');
title('\rm Raw gyro x');xlabel('Time (s)');ylabel('Gyro x (deg/s)');grid on;
subplot(3,1,2);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_y*R2D, 'k');
title('\rm Raw gyro y');xlabel('Time (s)');ylabel('Gyro y (deg/s)');grid on;
subplot(3,1,3);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_z*R2D, 'k');
title('\rm Raw gyro z');xlabel('Time (s)');ylabel('Gyro z (deg/s)');grid on;

%-->
figure;
subplot(3,1,1);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_x, 'k');hold on;
title('\rm Raw acc x');xlabel('Time (s)');ylabel('Acc x(m/s^2)');grid on;
subplot(3,1,2);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_y, 'k');
title('\rm Raw acc y');xlabel('Time (s)');ylabel('Acc y(m/s^2)');grid on;
subplot(3,1,3);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_z, 'k');
title('\rm Raw acc z');xlabel('Time (s)');ylabel('Acc z(m/s^2)');grid on;

%-->
figure;
subplot(3,1,1);plot(time_vector_sec-time_vector_sec (1), Euler_heading_value(1,1:end-1)*R2D, 'r');hold on;
plot(time_vector_sec-time_vector_sec (1), raw_imu_data(1:end-1,13)*R2D, '-','color','b','linewidth',1);grid on;
xlabel('Time (s)');ylabel('Heading(deg)');legend('EKF','Ground-truth');title('\rm Heading Estimation Results');
subplot(3,1,2);plot(time_vector_sec-time_vector_sec (1), Euler_roll_value(1,1:end-1)*R2D, 'r');hold on;grid on;
plot(time_vector_sec-time_vector_sec (1), raw_imu_data(1:end-1,11)*R2D, '-','color','b','linewidth',1);
xlabel('Time (s)');ylabel('Roll(deg)');legend('EKF','Ground-truth');title('\rm Roll Estimation Results');
subplot(3,1,3);plot(time_vector_sec-time_vector_sec (1), Euler_pitch_value(1,1:end-1)*R2D, 'r');hold on;grid on;
plot(time_vector_sec-time_vector_sec (1), raw_imu_data(1:end-1,12)*R2D, '-','color','b','linewidth',1);grid on;
xlabel('Time (s)');ylabel('Pitch(deg)');legend('EKF','Ground-truth');title('\rm Pitch Estimation Results');

%--> EKF vs. Noisy Updates
figure;
subplot(3,1,1);plot(time_vector_sec-time_vector_sec (1), mag_true_north_heading*R2D, '-','color','b','linewidth',1);grid on;
hold on;plot(time_vector_sec-time_vector_sec (1), Euler_heading_value(1,1:end-1)*R2D, '-.g','LineWidth',3);
xlabel('Time (s)');ylabel('Heading(deg)');legend('MAG-calculated','EKF-AHRS');title('\rm Heading MAG-calculated vs. EKF-AHRS');
subplot(3,1,2);plot(time_vector_sec-time_vector_sec (1), roll_quasi(1,1:end-1)*R2D, '-','color','b','linewidth',1);
hold on;plot(time_vector_sec-time_vector_sec (1), Euler_roll_value(1,1:end-1)*R2D, '-.g','LineWidth',3);grid on;
xlabel('Time (s)');ylabel('Roll(deg)');legend('Acc-calculated','EKF-AHRS');title('\rm Roll Acc-calculated vs. EKF-AHRS');
subplot(3,1,3);plot(time_vector_sec-time_vector_sec (1), pitch_quasi(1,1:end-1)*R2D, '-','color','b','linewidth',1);
hold on;plot(time_vector_sec-time_vector_sec (1), Euler_pitch_value(1,1:end-1)*R2D, '-.g','LineWidth',3);grid on;
xlabel('Time (s)');ylabel('Roll(deg)');legend('Acc-calculated','EKF-AHRS');title('\rm Pitch Acc-calculated vs. EKF-AHRS');

%--> Errors Plot
roll_error = (raw_imu_data(:,11)-Euler_roll_value')*R2D;
pitch_error = (raw_imu_data(:,12)-Euler_pitch_value')*R2D;
heading_error = (raw_imu_data(:,13)-Euler_heading_value')*R2D;

figure;
subplot(3,1,1);plot(time_vector_sec(1:end)-time_vector_sec(1), heading_error(1:end-1), 'k','LineWidth',2);
title('\rm AHRS-EKF Heading error (deg)');xlabel('Time (s)');ylabel('Error (deg)');grid on;ylim([-30 30]);
subplot(3,1,2);plot(time_vector_sec(1:end)-time_vector_sec(1), roll_error(1:end-1), 'k','LineWidth',2);
title('\rm AHRS-EKFRoll error (deg)');xlabel('Time (s)');ylabel('Error (deg)');grid on;ylim([-5 5]);
subplot(3,1,3);plot(time_vector_sec(1:end)-time_vector_sec(1), pitch_error(1:end-1), 'k','LineWidth',2);
title('\rm AHRS-EKF Pitch error (deg)');xlabel('Time (s)');ylabel('Error (deg)');grid on;ylim([-5 5]);

%-> Gyro Biases
figure;
subplot(3,1,1);plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_x_value(1:end-1)*R2D, '-.k','LineWidth',2);grid on;
xlabel('Time (s)');ylabel('Gyro bias (deg/s)');title('\rm Gyro X Bias(deg/s)');%ylim([-0.5 0]);
subplot(3,1,2);plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_y_value(1:end-1)*R2D, '-.k','LineWidth',2);grid on;
xlabel('Time (s)');ylabel('Gyro bias (deg/s)');title('\rm \rm Gyro Y Bias(deg/s)');%ylim([-0.5 0.3]);
subplot(3,1,3);plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_z_value(1:end-1)*R2D, '-.k','LineWidth',2);grid on;
xlabel('Time (s)');ylabel('Gyro bias (deg/s)');title('\rm \rm Gyro Z Bias(deg/s)');%ylim([-7 3]);

%--> Gyro Biases with STDV
figure;subplot(3,1,1);
Vertices(:,2) = [gyro_bias_x_value(1:end-1)'*R2D+2*sqrt(gyro_bias_x_error_covariance(1:end-1))'; flipdim(gyro_bias_x_value(1:end-1)'*R2D-2*sqrt(gyro_bias_x_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_x_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('STDV','Gyro bias x');grid on;xlabel('Time (s)'); ylabel('Gyro x bias (deg/s)');title('\rm \rm Gyro bias(deg/s)');

subplot(3,1,2);
Vertices(:,2) = [gyro_bias_y_value(1:end-1)'*R2D+2*sqrt(gyro_bias_y_error_covariance(1:end-1))'; flipdim(gyro_bias_y_value(1:end-1)'*R2D-2*sqrt(gyro_bias_y_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_y_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('STDV','Gyro bias y');grid on;xlabel('Time (s)'); ylabel('Gyro y bias (deg/s)');title('\rm Gyro bias(deg/s)');

subplot(3,1,3);
Vertices(:,2) = [gyro_bias_z_value(1:end-1)'*R2D+2*sqrt(gyro_bias_z_error_covariance(1:end-1))'; flipdim(gyro_bias_z_value(1:end-1)'*R2D-2*sqrt(gyro_bias_z_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_z_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('STDV','Gyro bias z');grid on;xlabel('Time (s)'); ylabel('Gyro z bias (deg/s)');title('\rm Gyro bias(deg/s)');

%--> Quaternion error with SDTV
q0_error = q_ref(1:end-1,1)-a_value(1:end-1)';
q1_error = q_ref(1:end-1,2)-b_value(1:end-1)';
q2_error = q_ref(1:end-1,3)-c_value(1:end-1)';
q3_error = q_ref(1:end-1,4)-d_value(1:end-1)';

figure;subplot(3,1,1);
Vertices(:,2) = [q1_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q1_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q1_error','-.','color','red','linewidth',2);
title('\rm Quaternion error error:q1 component');xlabel('Time (s)');ylabel('Quaternion error error:a');
legend('STDV','Quaternion error error:(a)');grid on;

subplot(3,1,2);
Vertices(:,2) = [q2_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q2_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q2_error','-.','color','red','linewidth',1);grid on;
title('\rm Quaternion error error:q2 component');xlabel('Time (s)');ylabel('Quaternion error error:b');
legend('STDV','Quaternion error error:(b)');

subplot(3,1,3);
Vertices(:,2) = [q3_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q3_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q3_error','-.','color','red','linewidth',2);grid on;
title('\rm Quaternion error error:q3 component');xlabel('Time (s)');ylabel('Quaternion error error:c');
legend('STDV','Quaternion error error:(c)');

figure;
plot(time_vector, raw_mag_x, 'k','linewidth',3);hold on;
plot(time_vector, raw_mag_y, 'g','linewidth',3);
plot(time_vector, raw_mag_z, 'y','linewidth',3);
title('\rm Calibrated raw magnetic sensor measurements');
legend('Mag x','Mag y','Mag z');grid on;xlabel('Time (s)'); ylabel('mag (uT)');

return;

figure;
h16 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_mag_x, 'db');hold on;
h17 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_mag_y, '*g');
h18 = plot(time_vector_sec(1:end)-time_vector_sec (1), raw_mag_z, 'sr');
legend('mag x','mag y','mag z');grid on;xlabel('Time (s)'); ylabel('mag (uT)');
set(h16,'YDataSource','raw_mag_x');
set(h16,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h17,'YDataSource','raw_mag_y');
set(h17,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h18,'YDataSource','raw_mag_z');
set(h18,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');


figure;
Vertices(:,2) = [gyro_bias_x_value(1:end-1)'*R2D+2*sqrt(gyro_bias_x_error_covariance(1:end-1))'; flipdim(gyro_bias_x_value(1:end-1)'*R2D-2*sqrt(gyro_bias_x_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_x_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('STDV','Gyro bias x');grid on;xlabel('Time (s)'); ylabel('Gyro x bias (deg/s)');title('\rm Gyro bias(deg/s)');

figure;
Vertices(:,2) = [gyro_bias_y_value(1:end-1)'*R2D+2*sqrt(gyro_bias_y_error_covariance(1:end-1))'; flipdim(gyro_bias_y_value(1:end-1)'*R2D-2*sqrt(gyro_bias_y_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_y_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('STDV','Gyro bias y');grid on;xlabel('Time (s)'); ylabel('Gyro y bias (deg/s)');title('\rm Gyro bias(deg/s)');

figure;
Vertices(:,2) = [gyro_bias_z_value(1:end-1)'*R2D+2*sqrt(gyro_bias_z_error_covariance(1:end-1))'; flipdim(gyro_bias_z_value(1:end-1)'*R2D-2*sqrt(gyro_bias_z_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_z_value(1:end-1)'*R2D','-.','color','red','linewidth',1);
legend('STDV','Gyro bias z');grid on;xlabel('Time (s)'); ylabel('Gyro z bias (deg/s)');title('\rm Gyro bias(deg/s)');

set(h23,'YDataSource','gyro_bias_x_value(1:end-1)*R2D');
set(h23,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h24,'YDataSource','gyro_bias_y_value(1:end-1)*R2D');
set(h24,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');
set(h25,'YDataSource','gyro_bias_z_value(1:end-1)*R2D');
set(h25,'XDataSource','time_vector_sec(1:end)-time_vector_sec (1)');

figure;
h30 = patch('Faces',faces,'Vertices',CubePoints,'FaceVertexCData',hsv(6),'FaceColor','flat');
%grid on;
axis equal;
view(10,10);
set(h30,'XData',CubeXData,'YData',CubeYData,'ZData',CubeZData);
axis([-X X -X X -X X]);grid on;

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), mag_true_north_heading*R2D, 'b');grid on;title('\rm mag heading (deg)');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), roll_quasi(1:end-1)*R2D, 'b');grid on;title('\rm Acc roll (deg)');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), pitch_quasi(1:end-1)*R2D, 'b');grid on;title('\rm Acc pitch (deg)');


figure;
plot(time_vector_sec(1:50:end)-time_vector_sec (1), raw_imu_data(1:50:end-1,13)*R2D, '-','color','k','linewidth',1);grid on;
xlabel('Time (s)');hold on;
title('\rm Ground-truth Heading');ylabel('Heading(deg)');

figure;
plot(time_vector_sec(1:50:end)-time_vector_sec (1), raw_imu_data(1:50:end-1,11)*R2D, '-','color','k','linewidth',1);grid on;
xlabel('Time (s)');
title('\rm Ground-truth Roll');ylabel('Roll(deg)');

figure;
plot(time_vector_sec(1:50:end)-time_vector_sec (1), raw_imu_data(1:50:end-1,12)*R2D, '-','color','k','linewidth',1);grid on;
xlabel('Time (s)');ylabel('Pitch(deg)');
title('\rm Ground-truth Pitch');

figure;
plot(time_vector_sec(1:10:end)-time_vector_sec (1), mag_true_north_heading(1:10:end)*R2D, 'b');grid on;title('\rm mag heading (deg)');
hold on;
plot(time_vector_sec(1:50:end)-time_vector_sec (1), raw_imu_data(1:50:end-1,13)*R2D, '-','color','r','linewidth',1);grid on;
xlabel('Time (s)');ylabel('Heading(deg)');hold on;
legend('MAG-estimated heading','Ground-truth');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), pitch_quasi(1:end-1)*R2D, 'b');grid on;title('\rm Acc-estimated pitch (deg)');hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), Euler_pitch_value(1:end-1)*R2D, '-','color','r','linewidth',1);grid on;
xlabel('Time (s)');ylabel('Pitchdeg)');hold on;
legend('ACC-estimated pitch','EKF');

roll_error = (raw_imu_data(1:end-1,11)-Euler_roll_value(1:end-1)')*R2D;
pitch_error = (raw_imu_data(1:end-1,12)-Euler_pitch_value(1:end-1)')*R2D;
heading_error = (raw_imu_data(1:end-1,13)-Euler_heading_value(1:end-1)')*R2D;
time_vector = time_vector_sec(1:end)-time_vector_sec (1);

q0_error = q_ref(1:end-1,1)-a_value(1:end-1)';
q1_error = q_ref(1:end-1,2)-b_value(1:end-1)';
q2_error = q_ref(1:end-1,3)-c_value(1:end-1)';
q3_error = q_ref(1:end-1,4)-d_value(1:end-1)';

figure;
Vertices(:,2) = [q0_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q0_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q0_error','-.','color','red','linewidth',2);
title('\rm Quaternion error error:a component');xlabel('Time (s)');ylabel('Quaternion error error:a');
legend('STDV','Quaternion error error:(a)');grid on;

figure;
Vertices(:,2) = [q1_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q1_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q1_error','-.','color','red','linewidth',1);grid on;
title('\rm Quaternion error error:b component');xlabel('Time (s)');ylabel('Quaternion error error:b');
legend('STDV','Quaternion error error:(b)');

figure;
Vertices(:,2) = [q2_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q2_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q2_error','-.','color','red','linewidth',2);grid on;
title('\rm Quaternion error error:c component');xlabel('Time (s)');ylabel('Quaternion error error:c');
legend('STDV','Quaternion error error:(c)');

figure;
Vertices(:,2) = [q3_error+2*sqrt(b_error_covariance(1:end-1))'; flipdim(q3_error-2*sqrt(b_error_covariance(1:end-1))',1)];
Vertices(:,1) = [time_vector';flipdim(time_vector',1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q3_error','-.','color','red','linewidth',2);grid on;
title('\rm Quaternion error error:d component');xlabel('Time (s)');ylabel('Quaternion error error:d');
legend('STDV','Quaternion error error:(d)');


figure;plot(time_vector,roll_error);title('\rm Simulation Test: Nominal Design Point: roll error(deg)');xlabel('Time (s)');ylabel('roll error(deg)');grid on;
figure;plot(time_vector,pitch_error);title('\rm Simulation Test: Nominal Design Point: pitch error(deg)');xlabel('Time (s)');ylabel('pitch error(deg)');grid on;
figure;plot(time_vector,heading_error);title('\rm Simulation Test: Nominal Design Point: heading error(deg)');xlabel('Time (s)');ylabel('heading error(deg)');grid on;

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(gyro_sf_x_value(1:end-1)), 'k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(gyro_sf_y_value(1:end-1)), '-.k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(gyro_sf_z_value(1:end-1)), '--k');grid on;hold on;
legend('Gyro sf x','Gyro sf y','Gyro sf z');xlabel('Time (s)');ylabel('Scale Factor');title('\rm Gyro Scale Factor');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), acc_bias_x_value(1:end-1), 'k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), acc_bias_y_value(1:end-1), '-.k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), acc_bias_z_value(1:end-1), '--k');grid on;hold on;
legend('Acc bias x','Acc bias y','Acc bias z');xlabel('Time (s)');ylabel('Acc bias(m/s^2)');title('\rm Acc Bias (m/s^2)');

figure;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(acc_sf_x_value(1:end-1)), 'k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(acc_sf_y_value(1:end-1)), '-.k');grid on;hold on;
plot(time_vector_sec(1:end)-time_vector_sec (1), abs(acc_sf_z_value(1:end-1)), '--k');grid on;hold on;
legend('Acc sf x','Acc sf y','Acc sf z');xlabel('Time (s)');ylabel('Scale Factor');title('\rm Acc Scale Factor');
