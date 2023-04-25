function xb = nml(x, xmin, xmax)
%NML Normalizes a variable between 0 and 1 with respect to its maximum and
%minimum limits.
%
% Syntax:
%   xb = nml(x, xmin, xmax)
%
% Input arguments:
%   x - variable to be normalized
%   xmin - minimum limit of the variable x
%   xmax - maximum limit of the variable x
%
% Output arguments:
%   xb - normalized variable
%
% Example:
%   x = [1; 2; 3; 4; 5];
%   xmin = 1;
%   xmax = 5;
%   xb = nml(x, xmin, xmax);
%
%   xb =
%       0
%       0.25
%       0.5
%       0.75
%       1
%
% Author: 
% Sergio Andres Castaño Giraldo
% https://controlautomaticoeducacion.com/ 
%

% check if x, xmin, and xmax are row vectors and convert them to column
% vectors if needed
if isrow(x)
    x=x';
end
if isrow(xmin)
    xmin=xmin';
end
if isrow(xmax)
    xmax=xmax';
end

% normalize the variable x using the minimum and maximum limits
xb = (x - xmin) ./ (xmax - xmin);
