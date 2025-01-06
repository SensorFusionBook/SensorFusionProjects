%{
Gnss Positioning Estimation Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

function [ DeltaPseudo ] = CalculatePseudoRangeError( option , n_SV ,Pseudorange ,  SV_X ,SV_Y ,SV_Z , x0 )
%GET DELTA PSEUDORANGE of eachepoch (Residual )
if(option == 3)%(N,E,U) Coordinates
    %Convert to (Lat , Long , Height)
    [x0(1) , x0(2),x0(3) ] = NEU2plh(x0(1) , x0(2),x0(3));
    option = 2;
end
if(option ==  2)%(Lat , Long , Height) Coordinates
    %Convert to ( x , y, z)
    Xtemp  =plh2xyz(x0(1) , x0(2),x0(3));
    x0(1) = Xtemp(1);
    x0(2) = Xtemp(2);
    x0(3) = Xtemp(3);
end

DeltaPseudo = ones(n_SV,1);
c = 299792458;%Speed of light in a vacum

for n=1:n_SV
    x(1)= SV_X(n)-x0(1);
    x(2)= SV_Y(n)-x0(2);
    x(3)= SV_Z(n)-x0(3);
    b = x.^2;
    r = sqrt(b(1)+b(2)+b(3));
    p0 = r + ( x0(4));

    DeltaPseudo(n) = Pseudorange(n)- p0;
end

end

