%{
LiDAR odometry Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

function [ptCloud] = read_point_cloud(i,scan,Alpha, LidarFOV, L)

    xyzPoints = zeros(L,3);
    for j = 1:L+1
        beam_angle = LidarFOV(1) + (j-1)*Alpha;
        xyzPoints(j,1) = scan(i,j)*cos(beam_angle*pi/180);
        xyzPoints(j,2) = scan(i,j)*sin(beam_angle*pi/180);
        xyzPoints(j,3) = 5.0;
    end

    ptCloud = pointCloud(xyzPoints);

end