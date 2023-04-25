function [g,h]=RestrGRef(X,Par)
global F
%% Realocate variables
N=Par.N;        %Horizon Prediction
Nu=Par.Nu;      %Control Prediction 
ny=Par.ny;      %Outputs
nu=Par.nu;      %Inputs
Pz=Par.Pz;      %discrete transfer function
Yref=Par.Yref;
Fo=Par.Fo;      %Utopic Point
nit=Par.nit;    %Tuning Horizon
w=Par.w;        %Weight of multi objective optimization
inK=Par.inK;
Xsp=Par.Xsp;
Temp=Par.Temp;
Ts=Pz.Ts;
dmin=Par.dmin;
nrm=Par.nrm;
lineal = Par.lineal;
if lineal ~= 1
    X0 = Pz.X0;
    XC = Pz.XC;
    u0 = Pz.u0;
    ub1 = Pz.ub1;
    lb1 = Pz.lb1;
end

%% Desition variables (Try to find the weights.)
if nrm==0
    delta=X(2:ny+1);   %Reference Weighting Parameter
    lambda=X(ny+2:ny+nu+1); %Control Weighting Parameter
else
    delta=Par.delta;
    lambda=X(2:ny+1);   %Control Weighting Parameter
end
%% Closed Loop Simulation with internal Model
%Tendría sentido minimizar cada salida por separado sin intervención de los
%acomplamentos del sistema MIMO, la aproximación de cada salida con su
%referencia sería cada uno de los objetivos independeintes donde la
%entradas se deben mover para tratar de alcanzar la referencia estudiando
%cada salida por separado (Esto se cumple? se esta estudiando cada
%objetivo?). Al final la curva de pareto será trazada y colocaremos a todos
%los objetivos a competir entre ellos.
% sel=zeros(ny,1);
% for i=1:ny
%     sel(i)=1;
%     Xy1 = CloseLoopToolB(Par.Shell_SL,Pz,Xsp,N,Nu,delta,lambda,nit,Ts)';
%    
%     Xy(i,1:nit)=Xy1(i,1:nit);
%     sel(i)=0;
% end

if lineal ==1
    [Xy] = CloseLoopGPC(Pz,Xsp,N,Nu,delta,lambda,nit,inK,Ts);
else
    [Xy] = CloseLoopNMPC(X0,XC,u0,Xsp,N(1),Nu,delta,lambda,nit,ub1,lb1,inK,Ts);
end
% [Xy] = CloseLoopGPC(Pz,Xsp,N,Nu,delta,lambda,nit,inK,Ts);

%% Error Closed-loop vs Reference Trajectory
errFA=Xy(:,inK:end)-Yref(:,inK:end);
% error=Xsp(:,inK:end) - Xy(:,inK:end);
%Graficar
%  plot(Temp,Xy(1,:),Temp,Yref(1,:))
% plot(Temp(1:end-inK+1),errFA(1,:))
J1=(diag(errFA*errFA'));   
%% Integral Time-weighted Absolute Error 
J4=sum((errFA.^2.*Temp(inK:end))'); 	%ITSE (MF-MA)

J5=sum((errFA.^2.*(nit*(1./(Temp(2:end-inK+2)))))'); 
%% Objective Function
F=J1;
%X=[0.4505   18.9765   11.6862   89.4278   95.8866   96.4126   87.2037]
%F=[0.4505    0.4505    0.4505]
g=zeros(ny,1);
for i=1:ny
    g(i)= F(i) - w(i)*X(1) - Fo(i);
end
h=[];