function [B,A,d] = descompMPC(FT,varargin)
% By: Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/
% ______________________________________________________________________
% [B,A,d,D] = descomp(FT)
% FT = Transfer Function
% Q = Column where the measurable disturbance signals start (0 by default)
% D = Disturbance Polynomial
% Decomposes a transfer function into numerator (B), denominator (A), and delay (d) in discrete time. (Returns CELLS).
% If the numerator has the same order as the denominator in the discrete case, a delay is removed and incorporated into the numerator to maintain the sliding window logic of the predictive controller, ensuring always u(k-1).

ni = nargin;
Q = inf;

if ni == 2 % If the column for the measurable disturbance is defined
    Q = varargin{1};
end

[B, A] = tfdata(FT, 'v'); % Numerator and denominator of the process
[n, m] = size(FT);
d = FT.iodelay;

if n == 1 && m == 1
    b1 = B;
    a1 = A;
    clear A B;
    B{1} = b1;
    A{1} = a1; % Convert to cells in the SISO case
end

[y, u] = size(B);

for i = 1:y
    for j = 1:u
        if B{i,j}(1) ~= 0 && u < Q(1)
            d(i,j) = d(i,j) - 1;
            B{i,j} = [0 B{i,j}];
        end
        if dcgain(FT(i,j)) == 0
            d(i,j) = max(d(i,:));
        end
    end
end
end
