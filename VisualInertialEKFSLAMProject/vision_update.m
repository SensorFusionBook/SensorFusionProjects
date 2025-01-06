%{
Visual-Inertial Fusion Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

The algorithm is described in the following papers:

[1] S. Sheikhpour and M. M. Atia, "An Enhanced Visual-Inertial Navigation 
System Based on Multi-State Constraint Kalman Filter," 
IEEE 63rd Proceedings of Midwest Symposium on Circuits and Systems (MWSCAS)
, pp. 361-364,MA, USA, 2020, (https://ieeexplore.ieee.org/document/9184501)

[2] S. Sheikhpour and M. M. Atia "A Real-Time CPU-GPU Embedded
Implementation of a Tightly-Coupled Visual-Inertial Navigation System",
IEEE Access, Vol. 10, pp: 86384 - 86394, 17 August 2022
(https://ieeexplore.ieee.org/abstract/document/9858052)

%}

frame_id = floor(index/camera_frame_rate) + 1;

%--> Augmenting the covariance matrix with pose covariance of the new body pose
P_temp = [eye(size(P)); eye(3, size(P,1)); [zeros(4,6), eye(4, size(P,1)-6)]]*P*...
        ([eye(size(P)); eye(3, size(P,1)); [zeros(4,6), eye(4, size(P,1)-6)]]');
P = P_temp;

%--> Augmenting the indices list of the windowed camera poses with the recent pose
tracking_pose_indices = [tracking_pose_indices; frame_id];

%--> Features matching of the tracked and new ones
if (frame_id ~= 1)    

    %--> This part is commented as all features have been detected in a pre-processing step
    % image_curr = readimage(images, frame_id);            
    % points_current = detectSURFFeatures(image_curr);              
    % points_current = points_current.selectStrongest(max_num_dtct_features);
    % [features_current, points_current] = extractFeatures(image_curr, points_current, 'Method', 'SURF'); 
    % matched_indices = matchFeatures(tracking_features_status.features_vect, features_current, 'MatchThreshold', 70, 'Unique', true);  
    
    %--> load feature and points
    features_current = features_all{frame_id};
    points_current = points_all{frame_id};
    matched_indices = matchFeatures(tracking_features_status.features_vect, features_current, 'Method', 'Exhaustive', 'MatchThreshold', 70, 'Unique', true);  

    % Unmatching long tracking features
    keep_matched_indices = [];
    for i=1:size(matched_indices, 1)
        if (length(tracking_features_status.frames_group{matched_indices(i,1)}) <36)
            keep_matched_indices = [keep_matched_indices, i];
        end        
    end  
    matched_indices = matched_indices(keep_matched_indices,:);
    
    % Update tracking features status structure
    for j=1:length(matched_indices(:,1))
        tracking_features_status.frames_group{matched_indices(j,1)} = [tracking_features_status.frames_group{matched_indices(j,1)}, frame_id];
        tracking_features_status.img_coordinates{matched_indices(j,1)} = [tracking_features_status.img_coordinates{matched_indices(j,1)}; points_current.Location(matched_indices(j,2), :)];
        tracking_features_status.features_vect(matched_indices(j,1)) = features_current(matched_indices(j,2));
    end

    % Apply all updates if it is the last frame 
    if (frame_id == image_set_size)
        matched_indices = [1,1];
    end
    
    % Identify lost and new features indices
    lost_features_indices = setdiff(1:size(tracking_features_status.features_vect, 1), matched_indices(:,1))';
    new_features_indices = setdiff(1:points_current.Count, matched_indices(:,2))';
    selected_features_indices = [];    
    
    lost_features_count = length(lost_features_indices);
    new_features_count = length(new_features_indices);
    
    r_all = [];     % residual of all features lost in the current pose
    H_X_all = [];   % Measurment matrix (fixed states)of all features lost in the current pose
    inrange_feature_num = 0; % number of in range features (between min and max distance)
    
    cam_ints = eye(3,3);
    cam_ints(1,1) = cam_ints_value(index, 1);
    cam_ints(2,2) = cam_ints_value(index, 2);
    cam_ints(3,1) = cam_ints_value(index, 3);
    cam_ints(3,2) = cam_ints_value(index, 4);
    cam_ints_obj = cameraParameters('IntrinsicMatrix', cam_ints);
    
    for j=1:lost_features_count
        M_j = length(tracking_features_status.frames_group{lost_features_indices(j)}); % number of poses tracked the jth feature            
       
        if (M_j>=3)           
            % Re-estimate the position of the ith feature with all poses
            % Removing small std features
            feat_cord_std = std(tracking_features_status.img_coordinates{lost_features_indices(j)});
            % Triangulation of the first and last pose to initialize theta               
            R_LC1 = quat2rotm(cam_att_value(tracking_features_status.frames_group{lost_features_indices(j)}(1)*10-9, :));
            R_LCM = quat2rotm(cam_att_value(tracking_features_status.frames_group{lost_features_indices(j)}(M_j)*10-9, :));                
            R_C1_CM = R_LC1' * R_LCM;

            P_L_LC1 = cam_pos_value(tracking_features_status.frames_group{lost_features_indices(j)}(1)*10-9, :);
            P_L_LCM = cam_pos_value(tracking_features_status.frames_group{lost_features_indices(j)}(M_j)*10-9, :);

            P_CM_CMC1 = (R_LCM')*(P_L_LC1 - P_L_LCM)';                
            
            stereoParams = stereoParameters(cam_ints_obj,cam_ints_obj,R_C1_CM,P_CM_CMC1);
            P_C1_f_init = triangulate(tracking_features_status.img_coordinates{lost_features_indices(j)}(1, :),...
                                      tracking_features_status.img_coordinates{lost_features_indices(j)}(M_j, :),stereoParams);
                                                              
            theta_init = [P_C1_f_init(1)/P_C1_f_init(3); P_C1_f_init(2)/P_C1_f_init(3); 1/P_C1_f_init(3)];  % initial guess
                                    
            theta = theta_init;
            delta_theta = [100; 100; 100];
            r_M_j = 100.*ones(2*M_j, 1);
            J_r_M_j = zeros(2*M_j, 3);

            idx_1st = tracking_features_status.frames_group{lost_features_indices(j)}(1)*10-9;
            iter = 0;
            delta_z = 100;
           
            % First trial of Gauss Newton to find inliers
            while (delta_z> 1e-6 && iter<20)     % terminate GN if max iteration is reached or change in depth is less than 0.5cm                       
               for i=1:M_j                   
                   idx_ith = tracking_features_status.frames_group{lost_features_indices(j)}(i)*10-9;

                   % Residual vector
                   r_M_j(2*i-1:2*i, :) = r_fun(...
                           a_value(idx_1st), b_value(idx_1st), c_value(idx_1st), d_value(idx_1st), a_value(idx_ith), b_value(idx_ith), c_value(idx_ith), d_value(idx_ith),...
                           cam_exts_value(index, 1), cam_exts_value(index, 2), cam_exts_value(index, 3), cam_exts_value(index, 4), cam_exts_value(index, 5), cam_exts_value(index, 6), cam_exts_value(index, 7),...
                           theta(2), theta(3), theta(1), cam_ints_value(index, 1), cam_ints_value(index, 2), cam_ints_value(index, 3), cam_ints_value(index, 4),...
                           pos_east_value(idx_1st), pos_down_value(idx_1st), pos_east_value(idx_ith),  pos_down_value(idx_ith), pos_north_value(idx_1st), pos_north_value(idx_ith),... 
                           tracking_features_status.img_coordinates{lost_features_indices(j)}(i, 1), tracking_features_status.img_coordinates{lost_features_indices(j)}(i, 2));

                   % Jacobian of the residual
                   J_r_M_j(2*i-1:2*i, :) = J_r_fun(...
                           a_value(idx_1st), b_value(idx_1st), c_value(idx_1st), d_value(idx_1st), a_value(idx_ith), b_value(idx_ith), c_value(idx_ith), d_value(idx_ith),...
                           cam_exts_value(index, 1), cam_exts_value(index, 2), cam_exts_value(index, 3), cam_exts_value(index, 4), cam_exts_value(index, 5), cam_exts_value(index, 6), cam_exts_value(index, 7),...
                           theta(2), theta(3), theta(1), cam_ints_value(index, 1), cam_ints_value(index, 2),...
                           pos_east_value(idx_1st), pos_down_value(idx_1st), pos_east_value(idx_ith), pos_down_value(idx_ith), pos_north_value(idx_1st), pos_north_value(idx_ith));                                             
               end

               % Gauss-Newton minimization to estimate P_f_j_0 
               J_temp = J_r_M_j'*J_r_M_j;               
               if (rcond(J_temp)>1e-6)  % check for ill-conditioned jacobian
                   delta_theta = (J_temp)\(J_r_M_j'*r_M_j);
                   delta_z = 1/theta(3);
                   theta = theta - (delta_theta);
                   delta_z = abs(delta_z - (1/theta(3)));
                   iter = iter + 1;
               else                   
                   theta(3) = 100;
                   break;
               end
            end
           
            % Outlier Rejection
            resi2 = (r_M_j.*r_M_j);
            sum_resi2 = zeros(length(resi2)/2,1);
            for temp_i = 2:2:length(resi2)
                sum_resi2(temp_i/2) = sum(resi2(temp_i-1:temp_i));
            end

           % Check if the feature is in range 2m to 16m far from the camera and the normalized error is not large            
            theta_std = sqrt(((r_M_j'*r_M_j)/(2*M_j-3))*inv(J_temp));
            prho_std = theta_std(3,3);
            Z_std1 = ((1/theta(3))^2)*theta_std(3,3);
            Z_std = Z_std1*sqrt(1+2*((1/theta(3))^2)*(prho_std^2));    
            
            residual_norm = (r_M_j'*r_M_j)/(2*M_j);
            f_z = 1/theta(3);
            if((f_z>2) && (f_z<furthest_depth) && (Z_std<max_depth_std))                 
               % Transform the feature position to the L frame
               P_L_Lfj = quat2rotm(cam_att_value(idx_1st, :))*[theta(1)/theta(3); theta(2)/theta(3); 1/theta(3)] + cam_pos_value(idx_1st, :)';                        
               
               H_cam_X_fixed_M_j = zeros(2*M_j, fixed_state_list_length+(7*length(tracking_pose_indices)));
               H_cam_f_M_j = zeros(2*M_j, 3);
                
               pose_indices_in_H = find(ismember(tracking_pose_indices, tracking_features_status.frames_group{lost_features_indices(j)}));
               
               idx_1st = tracking_features_status.frames_group{lost_features_indices(j)}(1)*10-9;
               
               for i=1:M_j                                    
                   idx_ith = tracking_features_status.frames_group{lost_features_indices(j)}(i)*10-9;
                   
                   % Filling the fixed states part of H_X matrix
                   H_cam_X_fixed_M_j(2*i-1:2*i, fixed_state_list_length-10:fixed_state_list_length) = H_cam_X_fixed_i_fun(...
                             a_value(idx_1st), b_value(idx_1st), c_value(idx_1st), d_value(idx_1st), a_value(idx_ith), b_value(idx_ith), c_value(idx_ith), d_value(idx_ith),...
                             cam_exts_value(index, 1), cam_exts_value(index, 2), cam_exts_value(index, 3), cam_exts_value(index, 4), cam_exts_value(index, 5), cam_exts_value(index, 6), cam_exts_value(index, 7),...
                             theta(2), theta(3), theta(1), cam_ints_value(index, 1), cam_ints_value(index, 2),...
                             pos_east_value(idx_1st), pos_down_value(idx_1st), pos_east_value(idx_ith), pos_down_value(idx_ith), pos_north_value(idx_1st), pos_north_value(idx_ith));
                   
                   % Filling the windowed states part of H_X matrix
                   H_cam_X_fixed_M_j(2*i-1:2*i, fixed_state_list_length+7*pose_indices_in_H(i)-6 : fixed_state_list_length+7*pose_indices_in_H(i)) = H_cam_X_pi_i_fun(...
                            a_value(idx_1st), b_value(idx_1st), c_value(idx_1st), d_value(idx_1st), a_value(idx_ith), b_value(idx_ith), c_value(idx_ith), d_value(idx_ith),...
                            cam_exts_value(index, 1), cam_exts_value(index, 2), cam_exts_value(index, 3), cam_exts_value(index, 4), cam_exts_value(index, 5), cam_exts_value(index, 6), cam_exts_value(index, 7),...
                            theta(2), theta(3), theta(1), cam_ints_value(index, 1), cam_ints_value(index, 2),...
                            pos_east_value(idx_1st), pos_down_value(idx_1st), pos_east_value(idx_ith), pos_down_value(idx_ith), pos_north_value(idx_1st), pos_north_value(idx_ith));                          
                          
                   % position of the jth feature in the ith camera frame
                   P_Cifj = (quat2rotm(cam_att_value(idx_ith, :))')*(P_L_Lfj - cam_pos_value(idx_ith, :)');
                   
                   % Filling the H_f matrix                                  
                   H_cam_f_M_j(2*i-1:2*i, :) = ((1/P_Cifj(3)).*[cam_ints_value(index, 1), 0, (-cam_ints_value(index, 1)*(P_Cifj(1)/P_Cifj(3)));...
                                                                  0                         , cam_ints_value(index, 2), (-cam_ints_value(index, 2)*(P_Cifj(2)/P_Cifj(3)))])*(quat2rotm(cam_att_value(idx_ith, :))');
                                      
                   % Residual vector  
                   r_M_j(2*i-1:2*i, :) = r_fun(...
                           a_value(idx_1st), b_value(idx_1st), c_value(idx_1st), d_value(idx_1st), a_value(idx_ith), b_value(idx_ith), c_value(idx_ith), d_value(idx_ith),...
                           cam_exts_value(index, 1), cam_exts_value(index, 2), cam_exts_value(index, 3), cam_exts_value(index, 4), cam_exts_value(index, 5), cam_exts_value(index, 6), cam_exts_value(index, 7),...
                           theta(2), theta(3), theta(1), cam_ints_value(index, 1), cam_ints_value(index, 2), cam_ints_value(index, 3), cam_ints_value(index, 4),...
                           pos_east_value(idx_1st), pos_down_value(idx_1st), pos_east_value(idx_ith),  pos_down_value(idx_ith), pos_north_value(idx_1st), pos_north_value(idx_ith),... 
                           tracking_features_status.img_coordinates{lost_features_indices(j)}(i, 1), tracking_features_status.img_coordinates{lost_features_indices(j)}(i, 2));             
               end
               
               % uncomment for CPU use only
               A = null(H_cam_f_M_j');
               r_j_mapped = A'*r_M_j;
               H_X_j_mapped = A'*H_cam_X_fixed_M_j;
                            
               % Chi-square test and outlier rejection
               r_j_cov = H_X_j_mapped*P*H_X_j_mapped' + pxl_std_value.^2;
               r_j_gama = r_j_mapped'*inv(r_j_cov)*r_j_mapped;
               
               if(r_j_gama < chi2inv(chi_perc,2*M_j-3))
                   r_all = [r_all; r_j_mapped];
                   H_X_all = [H_X_all; H_X_j_mapped];
                   
                   inrange_feature_num = inrange_feature_num + 1;  
                   
                   map_features_count = map_features_count + 1;
                   map_features_pos{map_features_count, 1} = single(P_L_Lfj');

                   selected_features_indices = [selected_features_indices; lost_features_indices(j)];
                  
               end              
            end
       end
    end
    

    % --> Apply update
    if(inrange_feature_num~=0)       
        
        R_cam = (pxl_std_value.^2).*eye(size(H_X_all,1)); 
        
        [H_Q, H_R] = qr(H_X_all);

        tmp_idx = find(sum(H_R,2)==0);
        
        if (~isempty(tmp_idx))
            H_X_all = H_R(1:tmp_idx(1)-1, :);                       
            R_cam = (pxl_std_value.^2).*eye(tmp_idx(1)-1);
            r_all =  H_Q(:, 1:tmp_idx(1)-1)'*r_all;
        end        

        K = (P*H_X_all')/(H_X_all*P*H_X_all'+R_cam);

        state_correction_vector = K*r_all;
        
        correct_fixed_states;        
        correct_windowed_states;
        

        % Correct covariance
        P = (eye(size(H_X_all,2)) - K*H_X_all)*P*((eye(size(H_X_all,2)) - K*H_X_all)')+K*R_cam*(K');
        
        if (isreal(P))
            %--> Fixing Positive-Definiteness of P
            P = (P+P')/2;
            [V, D] = eig(P); % Calculate the eigende composition of P matrix
            d= diag(D);
            d(d <= 1e-7) = 1e-7; % Set any eigenvalues that are lower than threshold "TH" to a fixed non-zero "small" value 
            P = V*diag(d)*V'; % Recalculate P matrix in its Positive Defenite variant
        else
            disp('imaginary noise covariance matrix');return;
        end
        P_fixed_history(index+1,:) = diag(P(1:fixed_state_list_length, 1:fixed_state_list_length));
        P_att_history(:,:,index+1) = P(7:10, 7:10);

    end

    update_feature_size(frame_id) = inrange_feature_num;

    %--> Replace the lost feature with new feature if there exists any
    tracking_features_status.frames_group = cat(1, tracking_features_status.frames_group{matched_indices(:,1)}, num2cell(frame_id.*ones(new_features_count, 1)));
    tracking_features_status.img_coordinates = cat(1, tracking_features_status.img_coordinates{matched_indices(:,1)}, num2cell(points_current.Location(new_features_indices, :), 2));
    tracking_features_status.features_vect = cat(1, tracking_features_status.features_vect(matched_indices(:,1), :), features_current(new_features_indices, :));
    
    %--> Removing lost poses enteries from windowed pose indices and covariance matrix
    keep_matched_indices = find(ismember(tracking_pose_indices, unique(horzcat(tracking_features_status.frames_group{:}))))';   
    
    P = P(sort([1:fixed_state_list_length,31+(keep_matched_indices*7),32+(keep_matched_indices*7),33+(keep_matched_indices*7),34+(keep_matched_indices*7),35+(keep_matched_indices*7), 36+(keep_matched_indices*7),37+(keep_matched_indices*7)]),...
          sort([1:fixed_state_list_length,31+(keep_matched_indices*7),32+(keep_matched_indices*7),33+(keep_matched_indices*7),34+(keep_matched_indices*7),35+(keep_matched_indices*7), 36+(keep_matched_indices*7),37+(keep_matched_indices*7)]));

    tracking_pose_indices = tracking_pose_indices(keep_matched_indices);

else % first frame processing

    %--> This part is commented as all features have been detected in a pre-processing step
    % image_curr = readimage(images, frame_id);         
    % points_current = detectSURFFeatures(image_curr);
    % points_current = points_current.selectStrongest(max_num_dtct_features);
    % [features_current, points_current] = extractFeatures(image_curr, points_current, 'Method', 'SURF');            
    % tracking_features_status.frames_group = num2cell(frame_id.*ones(points_current.Count, 1));
    % tracking_features_status.img_coordinates = num2cell(points_current.Location, 2);
    % tracking_features_status.features_vect = features_current; 

    %--> load feature and points
    features_current = features_all{frame_id};
    points_current = points_all{frame_id};
    tracking_features_status.frames_group = num2cell(frame_id.*ones(points_current.Count, 1));
    tracking_features_status.img_coordinates = num2cell(points_current.Location, 2);
    tracking_features_status.features_vect = features_current;       
end