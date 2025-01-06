%{
LiDAR Graph SLAM Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

function [scanob] = read_lidar_scan(i,scan,Alpha, LidarFOV, L)
    for k = 1:L
        beam_angle = LidarFOV(1) + (k-1)*Alpha;
        if isnan(scan(i,k))
            scan(i,k) = 0;
        end
        ranges(k) = scan(i,k);
        angles(k) = beam_angle;
    end
    scanob = lidarScan(ranges,angles*pi/180);
end