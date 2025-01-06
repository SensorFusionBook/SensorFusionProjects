%{
Inertial Navigation System Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

%}

function [r_out,v_out,q_out] = forward_kinematics(acc,gyr,r_in,v_in,q_in,lat,sampling_period_sec)
    C_NL  = [0 1 0;1 0 0;0 0 -1];                                           %DCM from L to N frame
    C_LN  = C_NL';                                                          %DCM from N to L frame
    we    = 2*pi/(24*60*60);                                                %Earth rotation rate (r/s)
    earth_a = 6378137;earth_f = 1/298.257223563;                            %Earth Shape Params
    earth_b = earth_a*(1-earth_f);earth_e2 = 1-(earth_b^2)/(earth_a^2);     %Earth Shape Params

    %--> Calculate Earth Carvature Rn and Rm
    Rn = earth_a/sqrt(1-earth_e2*sin(lat)*sin(lat)); 
    Rm = earth_a*(1-earth_e2)/((1-earth_e2*sin(lat)*sin(lat))^(1.5));    

    %--> Calculate transportation rate and earth rotation rate vectors
    vn   = v_in(2); ve   = v_in(1);
    w_N_EN_value = [-vn/(Rm + r_in(3)); ve/(Rn + r_in(3)); (ve*tan(lat))/(Rn + r_in(3))];
    w_N_IE_value = [0; we*cos(lat); we*sin(lat)];
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
    % g_o = 9.780318 * ( 1 + 5.3024e-03.*sin(lat).^2 - 5.9e-06.*(sin(2*lat)).^2 );
    % g = (g_o ./ (1 + (alt ./ sqrt(Rn*Rm))).^2);
    g_N = [0; 0; -9.8];

    V_N_dot_value = C_NL * C_LB_value * acc + g_N - cross((w_N_EN_value + 2*w_N_IE_value),v_in);
    v_out = v_in + V_N_dot_value*sampling_period_sec;

    %--> Advance position
    pos_dot = v_in - cross(w_N_EN_value,r_in);
    r_out = r_in + pos_dot*sampling_period_sec;
end