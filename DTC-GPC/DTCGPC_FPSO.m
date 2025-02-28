
clc
close all
clear all

%addpath('C:\Users\Sergio\Google Drive\UFSC\Controle Preditivo\1. Final MPC Code\MPC_Function')  
%% Define o processo do reator:
load('FPSO_Plant.mat')
Ts=2;           % Sampling Period
nit=1000;    % Number of iterations

Ps = H;
                    %Tiempo de Muestreo
%% Condicionamento
 Km=dcgain(Ps);
 [L,R] = CondMin(Km);
 L=sparse(L);
 R=sparse(R);
 Pse=L*Ps*R;
 Pz=c2d(Pse,Ts,'zoh');              %Processo discreto

[Bp,Ap,dp]=descompMPC(Pz);     %Descompone num, den e atraso de Pz em celdas
[my,ny]=size(Pz);             %Numero de entradas (ny) e Saídas (my)

%% Parametros de sintonia del GPC
%Encuentro cual es el retardo minimo en mi función de transferencia
dmin(1,my)=100;
dreal=Pz.iodelay;
for i=1:my
    dmin(i)=min(dp(i,:));
end


%% Fast Model
Gnz=Pz;
lmin = dreal-dmin'.*ones(my,ny);
lmin(lmin < 0) = 0;
Gnz.iodelay = lmin;
dnz=dp-diag(dmin)*ones(my,ny);


options = optimset('LargeScale','off');
qdis=1;        %Ponderacion Perturbacion
nq=1;              %Numero de perturbações
N(1:my)=10;         %Ventana de Predicción
Nu(1:ny-nq)=3;        %Horizonte de control

%Ponderación del seguimiento de referencia
delta(1:my)=0;               
%Ponderación de la acción de control
lambda=[50 1800 4]; 
%Ponderación de la variabl


%Variable de holgura (Epsilon)
Ep=1000*ones(1,my);  

%Matrices en bloque de los parametros de poderación
Ql=[];
for i=1:ny-nq
    Ql=blkdiag(Ql,lambda(i)*eye(Nu(i)));  %(lambda -> Acción de Control)
end
Ql=sparse(Ql);
Qd=[];
for i=1:my
    Qd=blkdiag(Qd,delta(i)*eye(N(i)));  %(delta  -> Seguimiento de Referencia)
end
Qd=sparse(Qd);

%Tamanho= 4 => restrições de controle maximos e minimos e incrementos maximos e minimos
%         2 => restrições de saida maximos e minimos
%Bam=sparse(eye(4*sum(Nu)+2*sum(N)));
%Prenche Hepsilon (Variavel de folga) com matrices nulas, correspondentes à
%ubicação das restrições das variáveis manipuladas (são 4) incrementos e
%controles máximos e mínimos
Heps=[];
for j=1:4
    for i=1:ny-nq
        Heps=blkdiag(Heps,0*eye(Nu(i)));  
    end
end
%depois prenche com relação a ponderação das variáveis de folga só para as
%restrições máximas e mínimas na saída (são 2)
rho=Heps;
for j=1:2
    for i=1:my
        Heps=blkdiag(Heps,eye(N(i)));  
        rho=blkdiag(rho,Ep(i)*eye(N(i)));
    end
end
Heps=sparse(Heps);
rho=sparse(rho);

%% Calculo de la ecuacion Diofantina
%Se obtiene la matriz B y A (Numerador y denominador sistema MIMO)
% la matriz A es el minimo común denominador del sistema
[Bt,A,na,nb] = BA_MIMO(Bp,Ap);

%Calculo de las Diofantinas
[E,En,F] = diophantineMIMO(A,N,0*dmin);

%Matriz bloque S con los coeficientes de la respuesta libre (polinomio F)
S=[];
for i=1:my
    S=blkdiag(S,F{i}(1:N(i),1:end)); 
end

%% Matriz de la Respuesta Forzada G
% [G] = MatG(Pz,N,Nu,dp);
[G,Gq,MG] = MatGQ(Pz,N,max(Nu)*ones(1,ny),dp,ny-nq+1);

%% Sistema Irrestrito
S1=G'*Qd*G+Ql;
M=((S1+S1')/2)\G'*Qd;               %Ganancia M
K(1,:)=M(1,:);                       %Tomo la primera columna unicamente (para control u1)
for i=1:ny-nq-1
    K(i+1,:)=M(sum(Nu(1:i))+1,:);    %Tomo valores de M desde [N(1)+1] una muestra despues del primer 
                                     %horizonte de control (para control u2)
end

%% Filtro Fr y S
% [So,Fr]=mimofilter(Pz,0.995,0.1,3);

beta=0.9997;
for j=1:my
    Fr(j,j)=zpk([],[beta beta],(1-beta)^2,Ts); 
end

%  Km1=dcgain(Fr);
%  [L1,R1] = CondMin(Km1);
%  Fr=L1*Fr*R1;
 
So=delay2z(Gnz)-delay2z(Fr)*delay2z(Pz);

if abs(pole(So)) <= 1.0
    disp('S(z) Stable')
else
    disp('S(z) Unstable')
end

%Determino el Vector con los controles pasados utilizando la funcion:
uG = deltaUFree(Bt,En,N,dnz);
Hp = cell2mat2(uG);   %Polinomio con controles Pasados
% duM=max(nb+dp); %GPC
duM=max(nb+dnz); %DTC-GPC
up=zeros(sum(duM),1); %Vector Controles Pasados

%% Cria vetores com as restricoes do SISTEMA

 InU=R(1:3,1:3)\[0.5 0.5 0.5]';  %Incremento de Controle
 Umx=R(1:3,1:3)\[10 10 10]';  %Controle Máximo (Hard Constrain)
 Umn=R(1:3,1:3)\-[10 10 10]';   %Controle Mínimo (Hard Constrain)
 Ymx=L*5*ones(14);  %Saída Máxima (Soft Constrain)
 Ymn=-Ymx;     %Saída Mínima (Soft Constrain)

 I=eye(sum(Nu)); %Matriz identidad 

 % Restriccion no Sinal de Controle e no Incremento de Controle -----------------%
 Triang=[];  
 T=ones(sum(Nu),1);
 j=1;
for i=1:ny-nq
     InUmax(j:j+Nu(i)-1,1)=InU(i)*ones(Nu(i),1); %Incremento maximo
     InUmin(j:j+Nu(i)-1,1)=-InU(i)*ones(Nu(i),1); %Incremento maximo
     u_max(j:j+Nu(i)-1,1)=Umx(i)*ones(Nu(i),1);  
     u_min(j:j+Nu(i)-1,1)=Umn(i)*ones(Nu(i),1);
     j=sum(Nu(1:i))+1;
     Triang=blkdiag(Triang,tril(ones(Nu(i))));
 end

%Restricao de Saída
 j=1;
 for i=1:my
     y_max(j:j-1+N(i),1)=Ymx(i)*ones(N(i),1);  
     y_min(j:j-1+N(i),1)=Ymn(i)*ones(N(i),1);
     j=sum(N(1:i))+1;
 end
 
I=sparse(I);
Triang=sparse(Triang);

%Restricoes: [incremento de Controle, Acao de control, Restricao de Saída]
a1=[I; -I;Triang; -Triang;G; -G];
H1=(G'*Qd*G+Ql);

%% inicializa parametros de Simulacion
%Inicializa vectores de los incrementos de control
delU(1:ny,1:nit) = 0;   
%Salidas MIMO
y(1:my,1:nit) = 0;
% Ru(1:my,1:nit)=0.00*wgn (1, nit, 1e-3 ).*ones(my,nit);
Ru(1:my,1:nit)=0;
%Referencias
r(1:my,1:nit) = 0;
re(1:my,1:nit) =L*r; %Referencia Escalonada
%Acciones de Control
u(1:ny,1:nit) = 0; 
%As dois ultimas realmente são perturbações mesuráveis e não ações de
%controle
% Specify the MD vector
tmd = 249; %entry time of the measured disturbance.
v = zeros(nq,nit);
v(:,tmd:nit) = 0.1;
v_filt = tf(1, [100 1]);
t = 0:Ts:(nit-1)*Ts;
mdv = lsim(v_filt, v, t);
u(4,:) = mdv; 
ue(1:ny,1:nit) = R\u; %Accion de control Escalonada 

Ref=zeros(sum(N),1); %Referencias
%Comienzo el Lazo de Control
for k=max(na)+1:nit
     %% Salida del proceso Real   
      t = 0:Ts:(k-1)*Ts;
      %Saída do Processo + Condicao Inicial + Ruido
      y=lsim(Ps,u(:,1:k),t)'; 
%       ye=L*y; %Condicionada      
%      yp = OptimalPredictor(Fr,So,ue,ye,k);
     yp = L * OptimalPredictor(Fr,So,u,y,k);
      %% Calculo de la respuesta libre
     Yd=[];
     for j=1:my
         Yd=[Yd yp(j,k:-1:k-na(j))];
     end
     for i=1:my
          Ref(sum(N(1:i-1))+1:sum(N(1:i)))=re(i,k);
     end
      %respuesta livre
     yf=Hp*up+S*Yd';
     
     j=1;
     for i=1:ny-nq
         ub(j:j+Nu(i)-1,1)=ue(i,k-1);
         j=sum(Nu(1:i))+1;
     end
      b1=[InUmax; -InUmin;u_max-T.*ub; T.*ub-u_min;y_max-yf; yf-y_min];
      % Variable de Folga 
     H = blkdiag(H1, rho);
     Fo1=(yf-Ref)'*Qd*G;
     
     Fo = [Fo1'; zeros(size(b1, 1), 1)]; % the linear part of the objective function remains the same
     a = [a1                 -Heps;...
         zeros(size(a1))     -Heps  ]; 
      % this corresponds to converting the hard constraints "Ax <= b" to soft constraints "Ax-y <= b"
      % also adds the constraints " y >= 0"  in the form of " - y <= 0"
      
%      a = [a1 -eye(size(b1, 1))];
     b = [b1;zeros(length(b1),1)]; 
     %% Calculo del Incremento de Control     
      %delU(:,k)=K*(Ref-yf); %Sem Restrição
      options = optimset('LargeScale','off','MaxFunEvals',30000);
      [x,fval,exitflag] = quadprog((H+H')/2,Fo,a,b,[],[],[],[],[],options);
      %Toma as primeiras posições que conformam o incremento de controle
      %ótimo
      delU(1,k)=x(1);
         for i=1:ny-1-nq
            delU(i+1,k)=x(sum(Nu(1:i))+1);
         end
        
         %As ultimas dois posições de delU na real corresponde as
         %perturbações pelo tanto sustituio esas posições pelo deltaQ ou
         %incremento de perturbação mesurável 
         delU(1+ny-nq:ny,k)=ue(1+ny-nq:ny,k)-ue(1+ny-nq:ny,k-1);
    %% Calculo la acao de Controle
     if k==1
        ue(:,k)=delU(:,k);        
     else
        ue(:,k)=ue(:,k-1)+ delU(:,k);
     end   
     
     u(:,k)=R*ue(:,k);
     
     %Actualizo vectores de Controles Pasados 
    for i=1:ny
        aux_1=up(sum(duM(1:i-1))+1:sum(duM(1:i))-1);
        up(sum(duM(1:i-1))+1:sum(duM(1:i)))=[delU(i,k); aux_1];
    end
    
end
%% Grafico los Resultados
%(debe se escoger que entada deseo graficar)
Ymx=L\Ymx;
Ymn=L\Ymn;
Umx=R(1:ny-nq,1:ny-nq)*Umx;
Umn=R(1:ny-nq,1:ny-nq)*Umn;

coloresG = [30, 112, 0]/255;
colorBand = [0.4940 0.1840 0.5560];
fontsize = 18;
y = y';
u=u';
figure
for i = 1:3
    subplot(3,1,i); 
    stairs(t,u(:,i),'color',coloresG,'linewidth',4),grid;
    ylabel(['$$u_' num2str(i) '$$'],'interpreter','latex')
    xlabel('time [min]','interpreter','latex')
    set(gca,'FontSize',fontsize,'TickLabelInterpreter','latex')
    hold on
    plot([t(1) t(end)],[Umn(i) Umn(i)],'--',[t(1) t(end)],[Umx(i) Umx(i)],'--','color',colorBand,'linewidth',2)
end
legend('Manipulated variables', 'Hard-constraints','interpreter','latex')

figure
for i = 1:7
    subplot(4,2,i); 
    hold on
    plot(t,y(:,i),'color',coloresG,'linewidth',4)
    hold on
    plot([t(1) t(end)],[Ymn(i) Ymn(i)],'--',[t(1) t(end)],[Ymx(i) Ymx(i)],'--','color',colorBand,'linewidth',2)
    grid;ylabel(['$$y_' num2str(i) '$$'],'interpreter','latex')
    set(gca,'FontSize',fontsize,'TickLabelInterpreter','latex' )
    xlabel('time [min]','interpreter','latex')
end
legend('Reference trajectory','Closed-loop response Compressor Power', 'Soft-constraints','interpreter','latex')

figure
for i = 8:14
    subplot(4,2,i-7); 
    hold on
    plot(t,y(:,i),'color',coloresG,'linewidth',4)
    hold on
    plot([t(1) t(end)],[Ymn(i) Ymn(i)],'--',[t(1) t(end)],[Ymx(i) Ymx(i)],'--','color',colorBand,'linewidth',2)
    grid;ylabel(['$$y_' num2str(i) '$$'],'interpreter','latex')
    set(gca,'FontSize',fontsize,'TickLabelInterpreter','latex' )
    xlabel('time [min]','interpreter','latex')
end
legend('Reference trajectory','Closed-loop response Interstage Pressure', 'Soft-constraints','interpreter','latex')
