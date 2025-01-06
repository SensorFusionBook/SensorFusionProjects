%{
Gnss Positioning Estimation Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

function EstimatedRxPos_ECEF= calculate_receiver_position(SatellitesPos_ECI,Pseudoranges,SatClckBiases,IonosphericErrors,TroposphericErrors)

    NumOfVisibleSatellites = length(SatellitesPos_ECI);
    threshold = 1; % was 20
    residuals = 50;
    InitialStateVector = [0;0;0;0];
    PseudoRangeError = vecnorm(SatellitesPos_ECI - InitialStateVector(1:3), 2);
    EstimationMatrix_H = zeros(NumOfVisibleSatellites, 4);
    while residuals > threshold
        b = Pseudoranges - PseudoRangeError(:) + SatClckBiases - IonosphericErrors - TroposphericErrors;
        for i = 1:NumOfVisibleSatellites
            EstimationMatrix_H(i, 1) = (InitialStateVector(1)-SatellitesPos_ECI(1,i))/PseudoRangeError(1,i);
            EstimationMatrix_H(i, 2) = (InitialStateVector(2)-SatellitesPos_ECI(2,i))/PseudoRangeError(1,i);
            EstimationMatrix_H(i, 3) = (InitialStateVector(3)-SatellitesPos_ECI(3,i))/PseudoRangeError(1,i);
            EstimationMatrix_H(i, 4) = 1;
        end
        StateCorrection = (EstimationMatrix_H'*EstimationMatrix_H)\EstimationMatrix_H'*b;
        residuals = norm(StateCorrection(1:3));
        InitialStateVector = InitialStateVector + StateCorrection;
        PseudoRangeError = vecnorm(SatellitesPos_ECI - InitialStateVector(1:3), 2);
    end
    EstimatedRxPos_ECEF = InitialStateVector(1:3)'; 
end