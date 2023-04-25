function xb = dnml(x,xmin,xmax)
% Function that denormalizes a variable with respect to its minimum and 
% maximum limits.
%
% Inputs:
% x     - Normalized variable
% xmin  - Minimum limit of variable
% xmax  - Maximum limit of variable
%
% Outputs:
% xb    - Denormalized variable
%
% Example:
% x = [0.5, 0.7, 0.3];
% xmin = [0.1, 0.2, 0.3];
% xmax = [0.9, 0.8, 0.7];
% xb = dnml(x, xmin, xmax);
%
% Author: 
% Sergio Andres Castaño Giraldo
% https://controlautomaticoeducacion.com/ 
%

% Ensure inputs are column vectors
if isrow(x)
    x=x';
end
if isrow(xmin)
    xmin=xmin';
end
if isrow(xmax)
    xmax=xmax';
end

% Denormalize the variable
xb = x .* (xmax - xmin) + xmin;
end
