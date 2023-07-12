function [S,Fr] = mimofilter(Pd,alfa,raio,kn)
% [S(z),Fr(z)] =  filtro_mimo(P(z),alfa,raio)
%
% Function to calculate the robustness filter Fr(z) and the stable predictor S(z).
%
% Pd:   defines the discrete multivariable process with delay
% alfa: defines the desired pole values
% raio: defines the maximum magnitude of the desired poles
% kn:   Numerator multiplicity (should be at least equal to the number of poles desired to be canceled - NOT implemented)
%
% by:
% Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/

% Establish the process representation
Pd.variable='z';                     % Set z as the variable of Pd
G=Pd;                                % Define G as the fast model
set(G,'inputdelay',0)
set(G,'outputdelay',0)
set(G,'iodelay',0)
h=Pd.Ts;

[m,n]=size(Pd);

dp=Pd.iodelay;
dmin(1:m)=100;
for i=1:m
    dmin(i)=min(dp(i,:));
end

% Obtain data for the Fr(z) filter circle

for i=1:m
    G2=1;
    for j=1:n
        if sum(G(i,j).num{1})~=0
            G2=minreal(G(i,j)*G2); 
        end
    end
    G2.iodelay=dmin(i);
    H(i)=G2;  
    if sum(H(i).num{1})==0
        Fr(i,i) = tf(1,1,h);  % Set Fr(i,i) to unity transfer function with the sampling time h
    else
        Fr(i,i) = minreal(filtro_siso(H(i),alfa,raio,kn));  % Calculate Fr(i,i) using the filtro_siso function
    end
end
Lo=dp-diag(dmin)*ones(m,n);
G.iodelay=Lo;
S=(ss(G)-ss(Fr)*ss(Pd));  % Calculate the stable predictor S(z)

%% Validation of the Filters
DG=round(dcgain(Fr)*10000);
if DG == eye(m)*10000
    disp('Fr(z) Static Gain OK')  % Check if the static gain of Fr(z) is unity for all diagonal elements
else
    disp('Fr(z) Static Gain Wrong')
end

if pole(S) < 1.0
    disp('S(z) Stable')  % Check if the poles of S(z) are within the unit circle
else
    disp('S(z) Unstable')
end
