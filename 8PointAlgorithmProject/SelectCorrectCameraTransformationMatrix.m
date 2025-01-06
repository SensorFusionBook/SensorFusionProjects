%{
8-Points Algorithm for Camera Motion Estimation
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

Acknowledgements
The data used in this project is based on the MATLAB camera calibration example in the following links:
https://www.mathworks.com/help/vision/camera-calibration.html
https://www.mathworks.com/help/vision/ug/evaluating-the-accuracy-of-single-camera-calibration.html

%}

function [P] = SelectCorrectCameraTransformationMatrix(y1,y2,T_cam1_to_cam2, K)

M = length(y1(1,:));
y1_hat = K\y2(:,1:M);
y2_hat = K\y1(:,1:M);

min_error = 10000;

for i = 1:4
    R = T_cam1_to_cam2(1:3,1:3,i);
    T = T_cam1_to_cam2(:,4,i);
    if abs(det(R) - 1) > 1e-6
        continue;
    end
    y2_hat_est = [R T]*[y1_hat;ones(1,M)];
    % One possible check is that the three elemens of lambda2 are positive and close to each other.
    % lambda = y2_hat_est./y2_hat;
    % Another possible check is reprojection error (|y2_hat-y2_hat_est|)
    error = sqrt(sum((y2_hat_est-y2_hat).^2));
    if mean(error) < min_error
        min_error = mean(error);
        P = T_cam1_to_cam2(:,:,i);
    end
end
