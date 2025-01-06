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

function T = get4PossibleSolutions(E)

    % Initialize the return 4 matrices 
    T = zeros(3,4,4);

    % Calculate single value decomposition of E
    [U,S,V] = svd(E);
    
    % Calculate the two possible rotations
    W = [0,-1,0;
        1,0,0;
        0,0,1];        
    
    R1 = U*W*V';
    R2 = U*W'*V';

    if abs(det(R1) - 1) > 1e-6
        R1(:,3) = -1*R1(:,3);
    end    
    if abs(det(R2) - 1) > 1e-6
        R2(:,3) = -1*R2(:,3);
    end

    % Calculate the two possible translations
    T1_skew = U*W*S*U';
    T2_skew = U*W'*S*U';
    T1 = [T1_skew(3,2); T1_skew(1,3); T1_skew(2,1)];
    T2 = [T2_skew(3,2); T2_skew(1,3); T2_skew(2,1)];
    
    % Calculate the 4 possible permutations of rotations and translations
    T(:,:,1) = [R1, T1];
    T(:,:,2) = [R1, T2];
    T(:,:,3) = [R2, T1];
    T(:,:,4) = [R2, T2];
end