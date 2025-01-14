figure;
hold on;
plot3(dataset.east,dataset.north,dataset.alt, 'LineWidth', 1, 'color', [0.93 .69 .13]);grid on;xlabel('east');ylabel('north');zlabel('alt');
plot3(IMU_pos(2,:),IMU_pos(1,:),-IMU_pos(3,:), 'LineWidth', 1, 'color', 'b');grid on;xlabel('east');ylabel('north');zlabel('alt');
view(-60,30);title('3D Trajectory showing Ground Truth, Noisy Updates, and EKF output');
h_ekf_position  = plot3(pos_east_value(1:10:index), pos_north_value(1:10:index), pos_alt_value(1:10:index), 'color', 'r','LineWidth', 2);hold on;grid on;
h_map = plot3(map_features_pos_mat(1:map_features_count, 2), map_features_pos_mat(1:map_features_count, 1), -map_features_pos_mat(1:map_features_count, 3), 'k+', 'MarkerSize', 5);
h_cov  = plot3(pos_east_value(index)+sqrt(P(2,2)).*cos(-pi:0.2:pi), pos_north_value(index)+sqrt(P(1,1)).*sin(-pi:0.2:pi), (20+pos_alt_value(index)).*ones(size((-pi:0.2:pi),2),1), 'g-','LineWidth', 2);hold on;grid on;
set(h_ekf_position,'YDataSource','pos_north_value(1:10:index)');
set(h_ekf_position,'XDataSource','pos_east_value(1:10:index)');
set(h_ekf_position,'ZDataSource','pos_alt_value(1:10:index)');
set(h_map, 'XDataSource','map_features_pos_mat(1:map_features_count, 2)');
set(h_map, 'YDataSource','map_features_pos_mat(1:map_features_count, 1)');
set(h_map, 'ZDataSource','-map_features_pos_mat(1:map_features_count, 3)');
set(h_cov,'XDataSource','pos_east_value(index)+sqrt(P(2,2)).*cos(-pi:0.2:pi)');
set(h_cov,'YDataSource','pos_north_value(index)+sqrt(P(1,1)).*sin(-pi:0.2:pi)');
set(h_cov,'ZDataSource','pos_alt_value(index).*ones(size((-pi:0.2:pi),2),1)');
h_vehicle = patch('Faces',faces,'Vertices',CubePoints,'FaceVertexCData',hsv(10),'FaceColor','flat');
set(h_vehicle,'XData',CubeXData,'YData',CubeYData,'ZData',CubeZData);
xlabel('EP(m)');ylabel('NP(m)');zlabel('Height(m)');
axis([min(dataset.east-20) max(dataset.east+20) min(dataset.north-20) max(dataset.north+20) -100 300]);
legend('Ground truth', 'IMU stand-alone solution', 'Visual-inertial EKF','Map feature points', 'Covariance');
