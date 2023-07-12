function [Fr] = filtro_siso(Pd,alfa,raio,kn)
%
% [Fr] = filtro_siso(Pd,alfa,raio,kn)
%
% Function to calculate the robustness filter Fr(z).
%
% Pd:   defines the discrete single-input single-output process with delay
% alfa: defines the desired pole values
% raio: defines the maximum magnitude of the desired poles
% kn:   Numerator multiplicity (should be at least equal to the number of poles desired to be canceled)
%
% by:
% Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/

% Establish the process representation
G=Pd;                                % Define G as the fast model
set(G,'inputdelay',0)
set(G,'outputdelay',0)
set(G,'iodelay',0)
d=Pd.iodelay; 

h=Pd.Ts;
lista_p=[1];

polos=pole(G);                  % Obtain the poles of G(z)
p_ind=polos(find(abs(polos)>=raio))';  % Obtain the unwanted poles of G(z)
nm=length(p_ind);
nk=nm;
pd=0; % Increment size of A if there is no delay
if d==0 
    pd=2; % A should be incremented by 2 to complete the order on the right side
          % which is always one order less than the term on the left side.
    nk=nk+pd;
end
lista_p=[lista_p p_ind];        % List of unwanted poles

px=poly(lista_p);
% ordem=length(lista_p); 

% Denominator of the filter
Dr=1;
for i=1:nk
    Dr=conv(Dr,[1 -alfa]);
end
ordem=(length(Dr)-1)+d; % Maximum order of the left side term (Dr z^d)

pn=1; % Position of the pole in px
lpx=length(px); % Length of poles px

% I will have one extra unknown of the order
A=zeros(ordem+1,ordem+1+pd); % Create the matrix with the coefficients of the unknowns
ip=1; % Used to increase one row per column in the matrix A (Sylvester)
for j=ordem+2-d:ordem+1+pd  % Start at column d+1
    
    for i=ip:ordem+1 % Start by placing the values of the poles to be eliminated
        if pn<=lpx   % in a Sylvester matrix form
            A(i,j)=px(pn);
            pn=pn+1;
        end
    end
    pn=1; % Reset the index that counts in the matrix of poles to be eliminated
    ip=ip+1; % Increase one column each time I place the total number of poles
    
end

% Fill the first columns of the matrix, which contain the coefficients of the numerator (Nr) of the filter (Dr z^d + Nr)
j=1;
for i=d+1:ordem+1
       A(i,j)=1; 
       j=j+1;
end

% Fill Matrix B with the desired poles of the filter
B=zeros(ordem+1,1);
B(1)=1;
B(2:length(Dr),1)=Dr(1,2:length(Dr));
% if d==0
%     B=px'-B;
%     A=-A;
% end

X=A\B; % Solve the system of equations

% Numerator of the filter
Nr=[];
for i=1:ordem+1-d
    Nr=[Nr X(i)];
end

sum(Nr)/sum(Dr);
if isempty(p_ind)
    Fr=tf(1,1,h);
else
    Fr=tf(Nr,Dr,h);
end

end
