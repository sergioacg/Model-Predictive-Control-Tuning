function [MG, MGc] = MatG(Ps, N, Nu, d)
% MatG - Construct the matrix G for MPC controller
%
% [MG, MGc] = MatG(Ps, N, Nu, d)
%
% Inputs:
%   - Ps: Transfer function matrix of the process (cell array)
%   - N: Prediction horizon for each output (vector)
%   - Nu: Control horizon for each input (vector)
%   - d: Input-output delays of the process (matrix or vector)
%
% Outputs:
%   - MG: Matrix G formed by concatenating the matrices G for each input-output pair
%   - MGc: Cell array containing the individual matrices G for each input-output pair
%
% This function constructs the matrix G for model predictive control (MPC) based on the
% given transfer function matrix Ps, prediction horizon N, control horizon Nu, and input-output
% delays d. The resulting matrix G is formed by concatenating the individual matrices G for
% each input-output pair, which are calculated using the step response of the corresponding
% transfer function and the given horizons and delays. The function returns both the matrix G
% and the individual matrices G in a cell array.
%
% Example:
% Ps = {[tf(1, [1 2 1]) tf(2, [1 1])], [tf([1 2], [1 2 1]) tf(1, [1 1])]};
% N = [3 4];
% Nu = [2 3];
% d = [1 2; 0 1];
% [MG, MGc] = MatG(Ps, N, Nu, d);
% disp(MG);
% disp(MGc);
% Outputs:
% MG = [1 2 0 0 0; 2 1 2 0 0; 1 2 1 2 0; 2 1 2 1 2]
% MGc = {[1 2 0; 2 1 2], [1 2 1; 2 1 2]}
%
% Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/

[s, e] = size(Ps);
Ts=Ps.Ts;
if e == 1
    dmin = d;
else
    dmin = min(d'); % Minimum delay vector
end

g = cell(s, e); % Step response matrices
H = cell(s, e); % Individual matrices G

for i = 1:s % Output y
    for j = 1:e % Input u
        g{i, j} = step(Ps(i, j), (N(i) + dmin(i)) * Ts); % Step response of Ps
        G = zeros(N(i), Nu(j)); % Initialize matrix G
        
        dk = d(i, j) - dmin(i) + 1; % Delay index
        
        % Warning message
        if dk >= N(i)
            % display('Warning! Effective delays in the MIMO system are much larger than the prediction horizon N. Increase the horizon N.');
        end
        
        c = 1; % Column index
        
        % Fill the matrix G column by column
        for k = 1:Nu(j)
            G(k:end, c) = g{i, j}(dmin(i) + 2:dmin(i) + N(i) - k + 2); % Construct matrix G
            c = c + 1; % Move to the next column of G
        end
        
        H{i, j} = G; % Store each matrix G in a cell
    end
end

MG = cell2mat(H); % Convert cell array to matrix
MGc = H; % Matrix G in cell array
end
