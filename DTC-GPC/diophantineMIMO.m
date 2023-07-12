%---------------------------------------------------------- %
%                                                           %
% Solution of the MIMO Diophantine equation                 %
% Ver: 2.0                                                          %
%---------------------------------------------------------- %
% [E,En,F] = diophantine(A,N,dmin)
% A = MIMO Denominator (Cell Matrix)
% N = Prediction Window
% E = Complete E diophantine polynomial
% En = All E polynomials
% F = F Diofantina polynomials
% dmin = Vector with minimum delays of the MIMO system

function [E,En,F] = diophantineMIMO(A,N,dmin)
   [p,m] = size(A); % Number of outputs (p) and number of inputs (m)
   for i = 1:p
       [En1,Fn] = diophantine(A{i,i},N(i),dmin(i)); % Calculation of the Diofantina function
       E{i} = En1(end,:); % E polynomial is the last row output by the function
       F{i} = Fn;
       En{i} = En1;
   end
end
