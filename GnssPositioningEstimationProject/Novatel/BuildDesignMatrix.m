%{
Gnss Positioning Estimation Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}

function [ H  ] = BuildDesignMatrix( n_SV ,  SV_X ,SV_Y ,SV_Z , x0 )
H =ones(n_SV,4);
for n = 1:n_SV
    x(1)= x0(1)-SV_X(n);
    x(2)= x0(2)-SV_Y(n);
    x(3)= x0(3)-SV_Z(n);

    b = x.^2;
    c = b(1)+b(2)+b(3);
    r = sqrt(c);
    H(n,1)= x(1)/r ;
    H(n,2)= x(2)/r ;
    H(n,3)= x(3)/r ;
    H(n,4)= 1 ;
end
end

