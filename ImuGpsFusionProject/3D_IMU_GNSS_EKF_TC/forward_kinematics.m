function [r_out,v_out,q_out,lat_out,lon_out] = forward_kinematics(acc,gyr,r_in,v_in,q_in,lat0,lon0,lat_in,lon_in,sampling_period_sec)
    C_NL  = [0 1 0;1 0 0;0 0 -1];                                           %DCM from L to N frame
    C_LN  = C_NL';                                                          %DCM from N to L frame
    we    = 2*pi/(24*60*60);                                                %Earth rotation rate (r/s)
    earth_a = 6378137;earth_f = 1/298.257223563;                            %Earth Shape Params
    earth_b = earth_a*(1-earth_f);earth_e2 = 1-(earth_b^2)/(earth_a^2);     %Earth Shape Params

    %--> Calculate Earth Carvature Rn and Rm
    Rn = earth_a/sqrt(1-earth_e2*sin(lat_in)*sin(lat_in)); 
    Rm = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat_in)*sin(lat_in))^(1.5));    

    %--> Calculate transportation rate and earth rotation rate vectors
    vn   = v_in(2); ve   = v_in(1);
    w_N_EN_value = [-vn/(Rm + r_in(3)); ve/(Rn + r_in(3)); (ve*tan(lat_in))/(Rn + r_in(3))];
    w_N_IE_value = [0; we*cos(lat_in); we*sin(lat_in)];
    w_L_IL_value = C_LN*(w_N_EN_value + w_N_IE_value);
    
    %--> Perform attitude quaternion mechanization
    a = q_in(1); b = q_in(2); c = q_in(3); d = q_in(4);

    att_quat_matrix  = [a -b -c -d; b a -d c; c d a -b; d -c b a];    

    gyro_quat_vector = [0;gyr];

    w_il_x = w_L_IL_value(1); w_il_y = w_L_IL_value(2); w_il_z = w_L_IL_value(3);
    
    w_il_quat_matrix = [0           -w_il_x     -w_il_y     -w_il_z;...
                    w_il_x         0        -w_il_z      w_il_y;...
                    w_il_y       w_il_z        0        -w_il_x;...
                    w_il_z      -w_il_y      w_il_x        0   ];

    quat_vector = [a;b;c;d];

    attitude_quat_dot = 0.5*att_quat_matrix*gyro_quat_vector - 0.5*w_il_quat_matrix*quat_vector;
      
    %--> Advance quaternion
    q_out = q_in + attitude_quat_dot*sampling_period_sec;
    
    %--> Normalize the quaternion
    q_out = q_out./norm(q_out);

    %--> Calculate Euler angles from Quaternion
    [A, p, r] = quat2angle(q_out','ZYX');
    
    %-->Reset quaternion in case -180/180 boundaries crossed
    q_out = angle2quat(A,p,r,'ZYX');
    q_out = q_out./norm(q_out);
    
    %--> Advance velocity
    C_BL_value = angle2dcm(A,p,r,'ZYX');
    C_LB_value = C_BL_value';

    %--> Calculate gravity. For simplicity it is fixed in this example
    g_N = [0; 0; -9.8];

    V_N_dot_value = C_NL * C_LB_value * acc + g_N - cross((w_N_EN_value + 2*w_N_IE_value),v_in);
    v_out = v_in + V_N_dot_value*sampling_period_sec;

    %--> Advance position (using quaternion method)
    %pos_dot = v_in - cross(w_N_EN_value,r_in);
    %r_out = r_in + pos_dot*sampling_period_sec;

    % Altitude equation is simple as follows
    r_out(3) = r_in(3) + v_out(3)*sampling_period_sec;

    
    % Get C_EN dcm matrix from lat and lon
    C_EN_value(1,1) = cos(lon_in);C_EN_value(1,2) = - sin(lon_in)*sin(lat_in);C_EN_value(1,3) = sin(lon_in)*cos(lat_in);
    C_EN_value(2,1) = 0; C_EN_value(2,2) = cos(lat_in); C_EN_value(2,3) = sin(lat_in);
    C_EN_value(3,1) = -sin(lon_in); C_EN_value(3,2) = -cos(lon_in)*sin(lat_in); C_EN_value(3,3) = cos(lon_in)*cos(lat_in);

    % convert position dcm to position quaternion
    pos_quat_vector = dcm2quat( C_EN_value' );
    a_pos = pos_quat_vector(1);b_pos = pos_quat_vector(2);c_pos = pos_quat_vector(3);d_pos = pos_quat_vector(4);

    % calculate rate of change of position quaternion as a function in w_N_EN_value
    pos_quat_dot_value = 0.5*[a_pos -b_pos -c_pos -d_pos; 
                           b_pos a_pos -d_pos c_pos;
                           c_pos d_pos a_pos -b_pos; 
                           d_pos -c_pos b_pos a_pos]*[0;w_N_EN_value];

    pos_quat_vector = pos_quat_vector + pos_quat_dot_value'*sampling_period_sec;
    pos_quat_vector = pos_quat_vector./norm(pos_quat_vector);

    % calculate new position dcm from position quaternion
    C_EN_value = quat2dcm( pos_quat_vector );
    C_EN_value = C_EN_value';
    
    %--> Calculate lat and lon from position DCM
    lon_out = atan2(C_EN_value(1,3),C_EN_value(3,3));
    lat_out = atan2(C_EN_value(2,3),sqrt(C_EN_value(2,1)^2+C_EN_value(2,2)^2));

    r_out(1) = (lon_out-lon0)*(Rn+r_out(3))*cos(lat_out);
    r_out(2) = (lat_out-lat0)*(Rm+r_out(3));
    
end