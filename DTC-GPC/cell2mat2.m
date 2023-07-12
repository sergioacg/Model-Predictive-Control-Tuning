function A = cell2mat2(B)
% cell2mat2 - Convert a cell array of matrices to a single matrix
%
% A = cell2mat2(B)
%
% Inputs:
%   - B: Cell array containing matrices
%
% Output:
%   - A: Matrix formed by concatenating the matrices in B
%
% This function takes a cell array of matrices and concatenates them into a single matrix.
% The resulting matrix A is formed by arranging the matrices from B in rows and columns
% according to their sizes. Empty elements in B are treated as empty matrices in A.
%
% Example:
% B = {[1 2; 3 4], [], [5; 6]};
% A = cell2mat2(B);
% disp(A);
% Output: [1 2 0 0; 3 4 0 0; 0 0 5; 0 0 6]
%
% Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/

[m,n] = size(B);
c1 = zeros(m,n);
f1 = c1;
col = 0; fil = 0;
n1 = 0; m1 = 0;

% Find dimensions of elements in the cell array
for i = 1:m
    for j = 1:n
        aux = size(B{i,j});
        col = col + aux(2);
        if aux(1) > fil
            fil = aux(1);
        end
        f1(i,j) = aux(1);
        c1(i,j) = aux(2);
    end
    if col > n1
        n1 = col;
    end
    m1 = m1 + fil;
    col = 0; fil = 0;
end

A = zeros(m1, n1);
c1m = [0, max(c1)];
f1T = f1';
f1m = [0, max(f1T)];

for i = 1:m
    for j = 1:n
        A(1 + sum(f1m(1:i)):f1(i,j) + sum(f1m(1:i)), 1 + sum(c1m(1:j)):c1(i,j) + sum(c1m(1:j))) = B{i,j};
    end
end
end
