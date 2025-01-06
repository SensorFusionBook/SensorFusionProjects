%{
Inertial Navigation System Project

Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.

%}

fprintf('V east error(m/s) = %.10f\n', sqrt(mean(ve_error.^2)));
fprintf('V north error(m/s) = %.10f\n', sqrt(mean((vn_error).^2)));
fprintf('V up error(m/s) = %.10f\n', sqrt(mean((vu_error).^2)));

fprintf('East error(m) = %.10f\n', sqrt(mean((E_error).^2)));
fprintf('North error(m) = %.10f\n', sqrt(mean((N_error).^2)));
fprintf('Alt error(m) = %.10f\n', sqrt(mean((U_error).^2)));

fprintf('Roll error(deg) = %.10f\n', sqrt(mean((roll_error).^2)));
fprintf('Pitch error(deg) = %.10f\n', sqrt(mean((pitch_error).^2)));
fprintf('Heading error(deg) = %.10f\n', sqrt(mean((heading_error).^2)));