for i = 1: length(ems_data.no_of_sat)
    sat_count(i) = ems_data.no_of_sat{i};
    sat_ime(i) = ems_data.time((i-1)*sampling_freq+1);
end

%--> 
figure;plot(sat_ime,sat_count,'LineWidth',2);grid on;title('Number of Sisible Satellites');
xlabel('time(s)');ylabel('Satellites');ylim([0 max(sat_count)+1]);

time_vector = time_vector_sec(1:end)-time_vector_sec (1);

%--> 
figure;
subplot(3,1,1);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_x*R2D, 'k');
title('raw gyro x');xlabel('time(s)');ylabel('gyro x(deg/s)');grid on;
subplot(3,1,2);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_y*R2D, 'k');
title('raw gyro y');xlabel('time(s)');ylabel('gyro y(deg/s)');grid on;
subplot(3,1,3);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_gyro_z*R2D, 'k');
title('raw gyro z');xlabel('time(s)');ylabel('gyro z(deg/s)');grid on;

%-->
figure;
subplot(3,1,1);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_x, 'k');hold on;
title('raw acc x');xlabel('time(s)');ylabel('acc x(m/s^2)');grid on;
subplot(3,1,2);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_y, 'k');
title('raw acc y');xlabel('time(s)');ylabel('acc y(m/s^2)');grid on;
subplot(3,1,3);plot(time_vector_sec(1:end)-time_vector_sec (1), raw_acc_z, 'k');
title('raw acc z');xlabel('time(s)');ylabel('acc z(m/s^2)');grid on;

%-->
figure;
subplot(3,1,1);plot(time_vector_sec-time_vector_sec (1), Euler_heading_value*R2D,  '-.r','LineWidth',2);hold on;
plot(time_vector_sec-time_vector_sec (1), ems_data.heading*R2D, '-','color','b','linewidth',1);grid on;
xlabel('time(s)');ylabel('Heading(deg)');legend('EKF','Ground Truth');title('Heading Estimation Results');
subplot(3,1,2);plot(time_vector_sec-time_vector_sec (1), Euler_roll_value*R2D, '-.r','LineWidth',2);hold on;grid on;
plot(time_vector_sec-time_vector_sec (1), ems_data.roll*R2D, '-','color','b','linewidth',1);
xlabel('time(s)');ylabel('Roll(deg)');legend('EKF','Ground Truth');title('Roll Estimation Results');
subplot(3,1,3);plot(time_vector_sec-time_vector_sec (1), Euler_pitch_value*R2D,  '-.r','LineWidth',2);hold on;grid on;
plot(time_vector_sec-time_vector_sec (1), ems_data.pitch*R2D, '-','color','b','linewidth',1);grid on;
xlabel('time(s)');ylabel('Pitch(deg)');legend('EKF','Ground Truth');title('Pitch Estimation Results');

%--> Errors Plot
roll_error = (roll_ref_vector - Euler_roll_value)*R2D;
pitch_error = (pitch_ref_vector - Euler_pitch_value)*R2D;
heading_error = angdiff(heading_ref_vector,Euler_heading_value)*R2D;
east_error = ems_data.east - EP_value;
north_error = ems_data.north - NP_value;
up_error = ems_data.h - alt_value;

figure;
subplot(3,1,1);plot(time_vector_sec(1:end)-time_vector_sec(1), heading_error, 'k','LineWidth',1);ylim([-20 20]);
title('Heading error (deg)');xlabel('time(s)');ylabel('Error (deg)');grid on;ylim([-30 30]);
subplot(3,1,2);plot(time_vector_sec(1:end)-time_vector_sec(1), roll_error, 'k','LineWidth',1);
title('Roll error (deg)');xlabel('time(s)');ylabel('Error (deg)');grid on;ylim([-5 5]);
subplot(3,1,3);plot(time_vector_sec(1:end)-time_vector_sec(1), pitch_error, 'k','LineWidth',1);
title('Pitch error (deg)');xlabel('time(s)');ylabel('Error (deg)');grid on;ylim([-5 5]);

figure;
subplot(3,1,1);
Vertices(:,2) = [east_error + 1*sqrt(east_pos_error_covariance'); flipdim(east_error-1*sqrt(east_pos_error_covariance'),1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector_sec(1:end)-time_vector_sec(1), east_error, 'k','LineWidth',2);
title('East error (m)');xlabel('time(s)');ylabel('Error (m)');grid on;
legend('STDV 1\sigma','Error');

subplot(3,1,2);
Vertices(:,2) = [north_error + 1*sqrt(east_pos_error_covariance'); flipdim(north_error-1*sqrt(east_pos_error_covariance'),1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector_sec(1:end)-time_vector_sec(1), north_error, 'k','LineWidth',2);
title('North error (m)');xlabel('time(s)');ylabel('Error (m)');grid on;
legend('STDV 1\sigma','Error');

subplot(3,1,3);
Vertices(:,2) = [up_error + 1*sqrt(east_pos_error_covariance'); flipdim(up_error-1*sqrt(east_pos_error_covariance'),1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector_sec(1:end)-time_vector_sec(1), up_error, 'k','LineWidth',2);
title('Alt error (m)');xlabel('time(s)');ylabel('Error (m)');grid on;
legend('STDV 1\sigma','Error');

%-> Gyro Biases
figure;
subplot(3,1,1);plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_x_value*R2D, 'k','LineWidth',2);grid on;hold on;
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.gyro_bias(1)*ones(1,2), '--b', 'LineWidth', 2);
xlabel('time(s)');ylabel('gyro bias (deg/s)');title('Gyro X Bias(deg/s)');legend('Estimated Bias','True Bias');
subplot(3,1,2);plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_y_value*R2D, 'k','LineWidth',2);grid on;hold on;
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.gyro_bias(2)*ones(1,2), '--b', 'LineWidth', 2);
xlabel('time(s)');ylabel('gyro bias (deg/s)');title('Gyro Y Bias(deg/s)');legend('Estimated Bias','True Bias');
subplot(3,1,3);plot(time_vector_sec(1:end)-time_vector_sec (1), gyro_bias_z_value*R2D, 'k','LineWidth',2);grid on;hold on;
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.gyro_bias(3)*ones(1,2), '--b', 'LineWidth', 2);
xlabel('time(s)');ylabel('gyro bias (deg/s)');title('Gyro Z Bias(deg/s)');legend('Estimated Bias','True Bias');

%--> Gyro Biases with STDV
figure;subplot(3,1,1);
Vertices(:,2) = [gyro_bias_x_value*R2D + 1*sqrt(gyro_bias_x_error_covariance'); flipdim(gyro_bias_x_value*R2D-1*sqrt(gyro_bias_x_error_covariance'),1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_x_value'*R2D','-.','color','red','linewidth',1);
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.gyro_bias(1)*ones(1,2), '--b', 'LineWidth', 2);hold on;
legend('STDV 1\sigma','gyro bias x','true bias');grid on;xlabel('time(s)'); ylabel('gyro x bias (deg/s)');title('Gyro Bias(deg/s)');

subplot(3,1,2);
Vertices(:,2) = [gyro_bias_y_value*R2D+1*sqrt(gyro_bias_y_error_covariance)'; flipdim(gyro_bias_y_value*R2D-1*sqrt(gyro_bias_y_error_covariance)',1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_y_value'*R2D','-.','color','red','linewidth',1);
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.gyro_bias(2)*ones(1,2), '--b', 'LineWidth', 2);hold on;
legend('STDV 1\sigma','gyro bias y','true bias');grid on;xlabel('time(s)'); ylabel('gyro y bias (deg/s)');title('Gyro Bias(deg/s)');

subplot(3,1,3);
Vertices(:,2) = [gyro_bias_z_value*R2D+1*sqrt(gyro_bias_z_error_covariance)'; flipdim(gyro_bias_z_value*R2D-1*sqrt(gyro_bias_z_error_covariance)',1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,gyro_bias_z_value'*R2D','-.','color','red','linewidth',1);
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.gyro_bias(3)*ones(1,2), '--b', 'LineWidth', 2);hold on;
legend('STDV 1\sigma','gyro bias z','true bias');grid on;xlabel('time(s)'); ylabel('gyro z bias (deg/s)');title('Gyro Bias(deg/s)');

%--> Acc Biases with STDV
figure;subplot(3,1,1);
Vertices(:,2) = [acc_bias_x_value + 1*sqrt(acc_bias_x_error_covariance'); flipdim(acc_bias_x_value-1*sqrt(acc_bias_x_error_covariance'),1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,acc_bias_x_value','-.','color','red','linewidth',1);
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.accel_bias(1)*ones(1,2), '--b', 'LineWidth', 2);hold on;
legend('STDV 1\sigma','acc bias x','true bias');grid on;xlabel('time(s)'); ylabel('acc x bias (m/s^2)');title('Acc X Bias(m/s^2)');

subplot(3,1,2);
Vertices(:,2) = [acc_bias_y_value + 1*sqrt(acc_bias_y_error_covariance)'; flipdim(acc_bias_y_value - 1*sqrt(acc_bias_y_error_covariance)',1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,acc_bias_y_value','-.','color','red','linewidth',1);
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.accel_bias(2)*ones(1,2), '--b', 'LineWidth', 2);hold on;
legend('STDV 1\sigma','acc bias y','true bias');grid on;xlabel('time(s)'); ylabel('acc y bias (m/s^2)');title('Acc Y Bias(m/s^2)');

subplot(3,1,3);
Vertices(:,2) = [acc_bias_z_value + 1*sqrt(acc_bias_z_error_covariance)'; flipdim(acc_bias_z_value -1*sqrt(acc_bias_z_error_covariance)',1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,acc_bias_z_value','-.','color','red','linewidth',1);
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.accel_bias(3)*ones(1,2), '--b', 'LineWidth', 2);hold on;
legend('STDV 1\sigma','acc bias z','true bias');grid on;xlabel('time(s)'); ylabel('acc z bias (m/s^2)');title('Acc Z Bias(m/s^2)');

%--> EKF vs. Noisy Measurements
figure;
subplot(3,1,1);
plot(ems_data.time, ems_data.east_noisy, 'r');hold on;grid on;
plot(ems_data.time, ems_data.east, 'b');
plot(ems_data.time, EP_value(1:end), 'k');
xlabel('time(s)');ylabel('east');title('East Position(m)');
legend('Noisy Measurements','Reference','EKF Solution');

subplot(3,1,2);
plot(ems_data.time, ems_data.north_noisy, 'r');hold on;grid on;
plot(ems_data.time, ems_data.north, 'b');
plot(ems_data.time, NP_value(1:end), 'k');
xlabel('time(s)');ylabel('north');title('North Position(m)');
legend('Noisy Measurements','Reference','EKF Solution');

subplot(3,1,3);
plot(ems_data.time, ems_data.h_noisy, 'r');hold on;grid on;
plot(ems_data.time, ems_data.h, 'b');
plot(ems_data.time, alt_value(1:end), 'k');
xlabel('time(s)');ylabel('Alt');title('Alt Position(m)');
legend('Noisy Measurements','Reference','EKF Solution');

%--> Quaternion error with SDTV
q0_error = q_ref(:,1)-a_value;
q1_error = q_ref(:,2)-b_value;
q2_error = q_ref(:,3)-c_value;
q3_error = q_ref(:,4)-d_value;

figure;subplot(3,1,1);
Vertices(:,2) = [q1_error + 1*sqrt(b_error_covariance'); flipdim(q1_error - 1*sqrt(b_error_covariance'),1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q1_error','-.','color','red','linewidth',2);
title('Quaternion Covariance : q1');xlabel('time(s)');ylabel('Quaternion error');
legend('STDV 1\sigma','q1 error');grid on;

subplot(3,1,2);
Vertices(:,2) = [q2_error + 1*sqrt(c_error_covariance'); flipdim(q2_error - 1*sqrt(c_error_covariance'),1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q2_error','-.','color','red','linewidth',1);grid on;
title('Quaternion Covariance : q2');xlabel('time(s)');ylabel('Quaternion error');
legend('STDV 1\sigma','q2 error');

subplot(3,1,3);
Vertices(:,2) = [q3_error + 1*sqrt(d_error_covariance'); flipdim(q3_error - 1*sqrt(d_error_covariance'),1)];
Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(time_vector,q3_error','-.','color','red','linewidth',2);grid on;
title('Quaternion Covariance : q3');xlabel('time(s)');ylabel('Quaternion error');
legend('STDV 1\sigma','q3 error');

figure;
%Vertices(:,2) = [receiver_clk_bias_value + 2*sqrt(receiver_clk_bias_error_covariance'); flipdim(receiver_clk_bias_value - 2*sqrt(receiver_clk_bias_error_covariance'),1)];
%Vertices(:,1) = [time_vector;flipdim(time_vector,1)];
%fill(real(Vertices(:,1)),real(Vertices(:,2)), [7 7 7]/8);hold on;
plot(ems_data.time,receiver_clk_bias_value, 'r', 'LineWidth',2); hold on; grid on; set(gca,'FontSize',12);
plot([ems_data.time(1), ems_data.time(end)], ems_data.noiseInfo.clk_bias*ones(1,2), '--b', 'LineWidth', 2);
legend('Receiver clock bias' , 'True bias');
