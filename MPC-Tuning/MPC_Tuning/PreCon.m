function R = PreCon(N,Nu)
% Verify if the prediction horizon is greater than the control horizon
%
% Syntax:
% R = PreCon(N, Nu)
%
% Inputs:
% N  - Prediction horizon.
% Nu - Control horizon.
%
% Outputs:
% R  - Result of the verification, true if the prediction horizon is 
%      greater than or equal to the control horizon, false otherwise.
%
% Example:
% R = PreCon([10 20 30], [5 10 20])
%
% Author:
% Sergio Andres Castaño Giraldo
% https://controlautomaticoeducacion.com/
% 

if min(N) > max(Nu) && all(N ~= 0) && all(Nu ~= 0)
    R = true;
else
    R = false;
end 
    

