%{
Demonstration of Allan Variance
Copyright (c) 2024 Mohamed Atia

This software is licensed under the Academic Use License.
Permission is granted for academic, educational, and non-commercial purposes only.
For more details, refer to the LICENSE file in the root directory of this repository.

DISCLAIMER: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
See LICENSE for details.
%}


function [param_value] = find_av_parameter(tau,adev,desired_slope,tau_value)
    tau_logscale = log10(tau);
    adev_logscale = log10(adev);
    logscale_slope = diff(adev_logscale) ./ diff(tau_logscale);
    [~, index_of_min] = min(abs(logscale_slope - desired_slope));
    C = adev_logscale(index_of_min) - desired_slope*tau_logscale(index_of_min);
    param_value = 10^(desired_slope*log(tau_value) + C);
end