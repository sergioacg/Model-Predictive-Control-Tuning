function [B,A,d] = descompMPC(FT,varargin)
% By:  Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/
% ______________________________________________________________________
%[B,A,d,D] = descomp(FT)
% FT = Função de Transferencia
% Q = Coluna onde começa os sinais de perturbação mesuraveis (0 por
% defeito)
% D Polinomio Perturbação
%descompone uma função de transferencia em numerador (B) denominador
%(A) e atraso (d) em tempo discreto. (Retorna CELDAS). Se o numerador
%apresenta mesmo ordem que o denominador no caso discreto, se retira um
%atraso e se incorpora no numerador para manter a logica da janela
%deslizante do controlador preditivo,garantir sempre u(k-1).

ni = nargin;
Q=inf;
if ni==2 %Si define a coluna da perturbação mesurável
    Q=varargin{1};
end
    [B,A]=tfdata(FT,'v');  %Numerador e denominador do processo
    [n,m]=size(FT);
    d=FT.iodelay;
    if n==1 && m==1
        b1=B;a1=A;clear A B;
        B{1}=b1;A{1}=a1; %Converte em celdas no caso SISO
    end
    
    [y,u]=size(B);
    for i=1:y
        for j=1:u
            if B{i,j}(1)~=0 && u<Q(1)
                d(i,j)=d(i,j)-1;
                B{i,j}=[0 B{i,j}];
            end
            if dcgain(FT(i,j))==0;
                d(i,j)=max(d(i,:));
            end
        end
    end
end