function [L,R,S] = CondMin(K)
% One way to determine which variables should be rescaled is through the
% calculation of the minimum conditioning, that is, to determine the matrices
% that pre- and post-multiply matrix K to obtain a minimum Ro (Ro*), that is:
%
%  Ro*(K) = Min Ro(L*K*R)
%           L,R
%  Ro: Conditioning of the matrix
%  Ro*: Minimum conditioning of the matrix
%  
% Considering L and R are diagonal matrices, the solution to the optimization
% problem above yields the inputs and outputs that should be rescaled, 
% respectively, since:
% 
%    ye = L * y        and     xe = R^-1 * x
%
%    where: ye (scaled output), y (process output), xe (scaled input), 
%           and x (process input)
%
% How to call the function:
%
%    [L,R,S] = CondMin(K)
%    K = Static Gains Matrix.
%    S = Minimum Conditioning Matrix --- S=cond(L*K*R)
%
% By: Sergio Andres Castaño Giraldo
% Rio de Janeiro
% 2017
% https://controlautomaticoeducacion.com/

[m,n]=size(K);
li=zeros(1,m+n);   % Lower limit of variables
ls=ones(1,m+n);   % Upper limit of variables
% li=[];   % Lower limit of variables
% ls=[];   % Upper limit of variables
A=[];            % Inequality matrix
B=[];            % Inequality equality
Ae=[];           % Equivalence matrix
Be=[];           % Equivalence equality
X0(1:m+n)=0.1;

% opContr = optimset('Algorithm','sqp','UseParallel',0,...
%                    'display','off','TolX',1e-6,'TolFun',1e-7,...
%                    'TolCon',1e-5,'MaxFunEvals',8000); 

J=@(X)funobj(X,K); % Objective function

% Minimize objective function
% [Xt,~,fminflag]=fmincon(J,X0,A,B,Ae,Be,li,ls,[],opContr);
[Xt,~,fminflag]=fmincon(J,X0,A,B,Ae,Be,li,ls,[]);

if exist('fminflag','var')
%     disp('fminflag')
    if fminflag == -2
        fprintf('No feasible point found \n')
    elseif fminflag == 0
        fprintf('Too many function evaluations or iterations \n')
    elseif fminflag == 1
        fprintf('Converged \n')
    else
        fprintf('Converged \n')
    end
  L=blkdiag(diag(Xt(1:m))*eye(m));
  R=blkdiag(diag(Xt(m+1:end))*eye(n));
  S=cond(L*K*R);
end

function [fobj] = funobj(X,K)
    
    [m,n]=size(K);
    L1=blkdiag(diag(X(1:m))*eye(m));
    R1=blkdiag(diag(X(m+1:end))*eye(n));
    fobj=cond(L1*K*R1);
