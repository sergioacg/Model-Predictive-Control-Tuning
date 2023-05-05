%% Calculations of tuning parameters for an NMPC controller
% In this case study, we will study how to obtain the tuning of an NMPC
% controller using the optimization algorithm proposed in the article:
%
% Giraldo, Sergio A. C., Príamo A. Melo, and Argimiro R. Secchi. 2022.
% "Tuning of Model Predictive Controllers Based on Hybrid Optimization"
% Processes 10, no. 2: 351. https://doi.org/10.3390/pr10020351
%
% This code is an adaptation to make it work directly with the
% Matlab toolbox and thus facilitate the distribution of the
% code for use by other researchers who wish to improve it.
%
% https://la.mathworks.com/help/mpc/ref/nlmpc.html
%
% Author:
% Sergio Andres Castaño Giraldo
% 2023
% https://controlautomaticoeducacion.com/
%

clc
clear all
close all
addpath('MPC_Tuning') % Adds the 'MPC_Tuning' folder to the Matlab path

%% ********* Code Configuration Flags ************
rest = true; % false: Without constraints; true: With constraints
caso = 1; % 1: Case 1 (fast); 2: Case 2 (slow)
nominal = true; % true: Nominal case; false: Model error case
lineal = false; % Non-Linear model used in MPCTuning

%% Simulation Time Parameters (Simulation Parameters)
Ts=0.05;        % Sampling Period
nit=60;        % Number of Iterations
nu = 2;         % Number of Inputs of the MIMO System
ny = 2;         % Number of Outputs of the MIMO System
nx = 3;         % Number of States of the MIMO System
dp =zeros(ny,nu); % System Delay
inK=4;

%% System Constraints
% Limits of Control Actions
Caupp = 6.0; % Maximum Limit of Reaction A
Calow = 0;   % Minimum Limit of Reaction A
Cbupp = 1.2; % Maximum Limit of Reaction B
Cblow = 0;   % Minimum Limit of Reaction B
Fupp  = 150;   % Maximum Feed Flowrate (l/h)
Flow  = 0;     % Minimum Feed Flowrate (l/h)
Qupp  = 150;   % Maximum Heat Exchanged (kJ/h)
Qlow  = 40;   % Minimum Heat Exchanged (kJ/h)

ub=[Fupp Qupp];  % Upper bound of the manipulated variables
lb=[Flow Qlow];  % Lower bound of the manipulated variables

xmin = [Calow;Cblow;Qlow]; % Lower bound of the controlled variables
xmax = [Caupp;Cbupp;Qupp]; % Upper bound of the controlled variables
umin = [Flow Qlow];
umax = [Fupp Qupp];

%% System Initial Condition
% Steady state of the system
CA0 = 5.1;          % Concentration of A [mol/l]
CB0 = 1.1163;       % Concentration of B [mol/l]
T0  = 130;          % Temperature [C]
X0 = [CA0 CB0 T0];

% Setting initial conditions for the inputs
u0 = [20 130];
ts = 0:Ts:(nit-1)*Ts;   % Simulation Time
options = optimset('display','off');
% model = @(x)vandevusse_model(ts,x,u0);
% X0 = fsolve(model,X0,options);
% Definir función anónima en una variable
model = @(t,x,u) vandevusse_model(t,x,u);
% Utilizar en fsolve
X0 = fsolve(@(x) model(ts, x, u0), X0, options);
x0 = X0;

%Define the second and third states as outputs
xc = [2, 3];
%Store in the structure for the MPCTuning algortihm
init.x0 = x0; init.xc = xc; init.u0 = u0;
init.integrator = @ode15s; %Integrator function for solve the model

%% Define the reference trajectory
%r(1,1:nit)=X0(1);  
r(1,1:nit)=X0(2);  r(1,10:nit)=1.0; r(1,20:nit)=1.0;
r(2,1:nit)=X0(3);  r(2,41:nit)=130;

% Normalizing limits between 0 and 1
% x0 = nml(X0,xmin,xmax);
% u0 = nml(u0,umin,umax);
% ub = nml(ub,umin,umax);
% lb = nml(lb,umin,umax);
   
%% Create NMPC object using Matlab Toolbox
% Define a matlab NMPC object with:
%   nx: number of states = 3
%   ny: number of outputs = 2
%   nu: number of inputs (all of them are manipulated variables) = 2
nlobj_proj = nlmpc(nx,ny,nu);
nlobj_proj.States(1).Name = 'Concentration of A'; nlobj_proj.States(1).Units = 'mol/l';
nlobj_proj.States(2).Name = 'Concentration of B'; nlobj_proj.States(2).Units = 'mol/l';
nlobj_proj.States(3).Name = 'Temperature'; nlobj_proj.States(3).Units = 'C';
nlobj_proj.OV(1).Name = 'Concentration of B'; nlobj_proj.OV(1).Units = 'mol/l';
nlobj_proj.OV(2).Name = 'Temperature'; nlobj_proj.OV(2).Units = 'C';
nlobj_proj.MV(1).Name = 'Feed Flowrate'; nlobj_proj.MV(1).Units = 'l/h';
nlobj_proj.MV(2).Name = 'Heat Exchanged'; nlobj_proj.MV(2).Units = 'kJ/h';

nlobj_proj.Ts = Ts;                                  % NMPC sample time

% Internal model of the NMPC
nlobj_proj.Model.StateFcn = @(x,u) nmpc_vandevusse_state(x, u);
% The output function of the prediction model relates the states and inputs 
% at the current control interval to the outputs. If the number of states 
% and outputs of the prediction model are the same, you can omit OutputFcn.
% Specify the output function for the controller. In this case, define 
% the second and third states as outputs.

nlobj_proj.Model.OutputFcn  = @(x,u) [x(2); x(3)];

nlobj_proj.Model.IsContinuousTime = true;
%No parameters; the parameters are defined explicit inside models
nlobj_proj.Model.NumberOfParameters = 0; 

%% NMPC Constraints
% Set constraints on the outputs, control inputs and their increments.
% since this is a square system 2x2
for i = 1:nu
    % Manipulated Variables
    nlobj_proj.MV(i).Min = lb(i);
    nlobj_proj.MV(i).Max = ub(i); 
    % Control increment
    nlobj_proj.MV(i).RateMin = -inf;   
    nlobj_proj.MV(i).RateMax = inf;
end
for i = 1:ny
    nlobj_proj.OV(i).Min = xmin(i+1);
    nlobj_proj.OV(i).Max = xmax(i+1);
end

%% Reference response for controller tuning
% Define the desired response of the controller, in this case, a first-order
% dynamics with unit gain is chosen to reach the setpoint. The time constant
% is varied to modify the speed at which the reference is reached.
% Pref=[tf(1,[0.05 1]),0;0,tf(1,[0.0875 1])]; %Fast
% Pref=[tf(1,[0.3 1]),0;0,tf(1,[0.4 1])]; %VERY Slow
Pref=[tf(1,[0.15 1]),0;0,tf(1,[0.26 1])]; %Slow
Prefz=c2d(Pref,Ts,'zoh'); %Transfer function with the desired dynamics

%% Specify the MD vector
mdv = zeros(nit,0); %400×0 empty double matrix

% Reference response for compare in GAM algorithm
% SetPoint
Xspref = r-row2col(x0(xc)); 

% This dynamic does not necessarily need to be linear, I am doing it
% linear for convenience only.
t = 0:Ts:(nit-1)*Ts;  % Time vector for simulation 
Yref=lsim(Pref,Xspref,t,'zoh')+ones(nit,ny).*x0(xc); % Simulate reference response using lsim function

%% Initial NMPC Tuning for the MPCTuning algorithm
N = 10; % Prediction Horizon
Nu = 2; % Control Horizon
delta = [100 1];  % tracking setpoint weight
lambda = [1e-5 ,1]; % control increment weight
nlobj_proj.PredictionHorizon = N;                    % Prediction horizon
nlobj_proj.ControlHorizon = Nu;                      % Control horizon
nlobj_proj.Weights.OutputVariables = delta;
nlobj_proj.Weights.ManipulatedVariablesRate = lambda;

%% MPC Tuning algorithm
% Aplicar el algoritmo para intentar encontrar los parámetros de sintonia del
% controlador MPC para la columna de destilacion de la Shell 3x3
w=[0.7 0.3]; %Pesos para la curva de pareto
tic
    [nlobj,~,delta,lambda,N,Nu,Fob] = MPCTuning(nlobj_proj,r,lineal,w,nit,Yref,mdv,5,4,model,init);
toc


%% Validate Custom Functions
% Define the time horizon for the simulation
time = [0,3];

% Validate custom functions
validateFcns(nlobj,x0,u0);

% Preallocate state and input vectors
X = zeros(nx,nit);
Y = zeros(ny,nit);
U = zeros(nu,nit);

% Set initial conditions
X(:,1) = x0';
Y(:,1) = x0(2:end)';
U(:,1) = u0';

% Compute the reference trajectory
t = 0:Ts:(nit-1)*Ts;
r1f = r(1,1:nit)-r(1,1);
r2f = r(2,1:nit)-r(2,1);
Yref1=lsim((Prefz),[r1f;r2f],ts)+X0(2:3);

[Y,U,yopt,uopt] = closedloop_toolbox_nmpc(nlobj,model,init,r,N,Nu,delta,lambda,nit);

% Plot simulation results
figure;
subplot(3,1,1);
plot(ts,r(1,:),'LineWidth',2);
hold on;
plot(t,Yref1(:,1),':',ts,Y(1,:),'LineWidth',2);
ylabel('Concentration of B [mol/l]');
xlabel('Time [h]');
title('Closed-loop NMPC Simulation Results');
legend('Setpoint','Output Reference','Output','Location','best');
grid on;

subplot(3,1,2);
plot(ts,r(2,:),'LineWidth',2);
hold on;
plot(t,Yref1(:,2),':',ts,Y(2,:),'LineWidth',2);
ylabel('Temperature [C]');
xlabel('Time [h]');
legend('Setpoint','Output Reference','Output','Location','best');
grid on;

subplot(3,1,3);
plot(ts,U(1,:),'LineWidth',2);
hold on;
plot(ts,U(2,:),'LineWidth',2);
ylabel('Control Inputs');
xlabel('Time [h]');
legend('Feed Flowrate','Heat Exchanged','Location','best');
grid on;
