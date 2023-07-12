% Example extracted from the book:
% Control of Dead-time Processes, Springer-Verlag, 2007
% by Julio Normey-Rico & Eduardo F. Camacho
%
%---------------------------------------------------------- %
%                                                           %
% Solution of the Diophantine equation                      %
%                                                           %
%---------------------------------------------------------- %
%[E,F] = diophantine(A,N,d)
% A = Denominator
% N = Prediction Horizon
% d = Transport delay

function[E,F] = diophantine(A,N,d)

%clear all

% Computes polynomials E(z^-1) e F(z^-1) 

% delta = 1-z^(-1)

delta = [1 -1];

% A = 1 + a1 z^(-1) + ... + a2 z(-na)

%A = [1 -0.8]

%A = [1 -0.905]

%A = [1 -1.8 0.81]

% Ã = Adelta

AD = conv(A,delta);

% note that nAD = n~a + 1

nAD = size(AD);
nAD = nAD(2);

% compute horizons

 N1 = d +1;
 N2 = d + N;

% Compute F(z^-1)

% inilialization vector f

f(1,:)= [1 zeros(1,nAD-2)];

% i = 0 ... nã-1

for j = 1: N2;

% Note that for i = 1 corresponds to f(j,0)

for i = 1:nAD-2
   f(j+1,i) = f(j,i+1)-f(j,1)*AD(i+1);
end
   f(j+1,nAD-1) = -f(j,1)*AD(nAD);
end

F = f(1+N1:1+N2,:);

% Computes E(z^-1)

E = zeros(N2);
e(1) = 1;               % for the special case  1/~A

E(1,1) = e(1);

for i = 2: N2
    e(i) = f(i,1);
    E(i,1:i)=e;
end

E = E(N1:N2,:);

% -------------------- %
% -------------------- %