%% Example of NMPC Implementation for the Van de Vusse Reactor
% This example demonstrates the implementation of a Nonlinear Model 
% Predictive Controller (NMPC) applied to the Van de Vusse reactor, a 
% well-known benchmark system in process control literature. 
%________________________________________________________________
% For more information, visit:
% Website: [Place the link here]
% YouTube Video: [Place the link here]
%________________________________________________________________
% Tuning Method:
% This controller was tuned using the following open-access scientific work:
% Giraldo, Sergio A. C., Príamo A. Melo, and Argimiro R. Secchi. 2022.
% "Tuning of Model Predictive Controllers Based on Hybrid Optimization"
% Processes 10, no. 2: 351. https://doi.org/10.3390/pr10020351
%________________________________________________________________
% Author:
% Sergio Andres Castaño Giraldo
% 2023
% https://controlautomaticoeducacion.com/
%________________________________________________________________
% Also, feel free to explore my premium courses on UDEMY:
% - Fundamentos en Instrumentación Industrial: https://bit.ly/FunInsInd
% - Simulink desde Cero: https://bit.ly/3a0W8Xr
% - LabVIEW desde Cero: https://bit.ly/LabView0
% - Curso de Controladores con PIC: http://bit.ly/Control_PIC
% - Curso de Controladores con Arduino: http://bit.ly/Control_Ardu
% - Applied Control System in Arduino: https://bit.ly/AppControlCAE
%________________________________________________________________

clc
clear all
close all

%% ********* Code Configuration Flags ************
rest = true; % false: Without constraints; true: With constraints
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

%% Define the second and third states as outputs
xc = [2, 3];

%% NMPC Scales
% set input and output range
Urange = umax-umin;
Yrange = xmax(xc)-xmin(xc);
Xrange = xmax - xmin;

%% System Initial Condition
% Steady state of the system
CA0 = 5.1;          % Concentration of A [mol/l]
CB0 = 1.1163;       % Concentration of B [mol/l]
T0  = 130;          % Temperature [C]
X0 = [CA0 CB0 T0];

%% Internal Model
u0 = [20 130];
ts = 0:Ts:(nit-1)*Ts;   % Simulation Time
options = optimset('display','off');
model = @(t,x,u) vandevusse_model(t,x,u);
% Utilizar en fsolve
X0 = fsolve(@(x) model(ts, x, u0), X0, options);
x0 = X0;

%Store in the structure for the MPCTuning algortihm
init.x0 = x0; init.xc = xc; init.u0 = u0;
init.integrator = @ode15s; %Integrator function for solve the model

%% Define the reference trajectory
%r(1,1:nit)=X0(1);  
r(1,1:nit)=X0(2);  r(1,10:nit)=1.0; r(1,20:nit)=1.0;
r(2,1:nit)=X0(3);  r(2,41:nit)=130;

%% Create NMPC object using Matlab Toolbox
% Define a matlab NMPC object with:
%   nx: number of states = 3
%   ny: number of outputs = 2
%   nu: number of inputs (all of them are manipulated variables) = 2
nlobj = nlmpc(nx,ny,nu);
nlobj.States(1).Name = 'Concentration of A'; nlobj.States(1).Units = 'mol/l';
nlobj.States(2).Name = 'Concentration of B'; nlobj.States(2).Units = 'mol/l';
nlobj.States(3).Name = 'Temperature'; nlobj.States(3).Units = 'C';
nlobj.OV(1).Name = 'Concentration of B'; nlobj.OV(1).Units = 'mol/l';
nlobj.OV(2).Name = 'Temperature'; nlobj.OV(2).Units = 'C';
nlobj.MV(1).Name = 'Feed Flowrate'; nlobj.MV(1).Units = 'l/h';
nlobj.MV(2).Name = 'Heat Exchanged'; nlobj.MV(2).Units = 'kJ/h';

nlobj.Ts = Ts;                                  % NMPC sample time

% Internal model of the NMPC
nlobj.Model.StateFcn = @(x,u) nmpc_vandevusse_state(x, u);
% The output function of the prediction model relates the states and inputs 
% at the current control interval to the outputs. If the number of states 
% and outputs of the prediction model are the same, you can omit OutputFcn.
% Specify the output function for the controller. In this case, define 
% the second and third states as outputs.

nlobj.Model.OutputFcn  = @(x,u) [x(2); x(3)];

nlobj.Model.IsContinuousTime = true;
%No parameters; the parameters are defined explicit inside models
nlobj.Model.NumberOfParameters = 0; 

%% NMPC Constraints
% Set constraints on the outputs, control inputs and their increments.
% since this is a square system 2x2
for i = 1:nu
    % Manipulated Variables
    nlobj.MV(i).Min = lb(i);
    nlobj.MV(i).Max = ub(i); 
    % Control increment
    nlobj.MV(i).RateMin = -inf;   
    nlobj.MV(i).RateMax = inf;
end
for i = 1:ny
    nlobj.OV(i).Min = xmin(i+1);
    nlobj.OV(i).Max = xmax(i+1);
end
for i = 1:nx
    nlobj.States(i).Min = xmin(i);
    nlobj.States(i).Max = xmax(i);
end

%% NMPC Scales Configuration in the nlobj
for i = 1:nu
    nlobj.MV(i).ScaleFactor = Urange(i);
end
% scale outputs
for i = 1:ny
    nlobj.OV(i).ScaleFactor = Yrange(i);
end
% scale states
for i = 1:nx
    nlobj.States(i).ScaleFactor = Xrange(i);
end

%% NMPC Tuning for the MPCTuning algorithm
% Tuning Algorithm: 
% https://github.com/sergioacg/Model-Predictive-Control-Tuning/tree/main/MPC-Tuning
N = 3; % Prediction Horizon
Nu = 2; % Control Horizon
delta = [0.0930 0.1133];  % tracking setpoint weight
lambda = [0.2460 0.1231]; % control increment weight
nlobj.PredictionHorizon = N;                    % Prediction horizon
nlobj.ControlHorizon = Nu;                      % Control horizon
nlobj.Weights.OutputVariables = delta;
nlobj.Weights.ManipulatedVariablesRate = lambda;

    % N = Tuning_Parameters.N; Nu = Tuning_Parameters.Nu;
    % delta = Tuning_Parameters.delta;
    % lambda = Tuning_Parameters.lambda;
    % nlobj = Tuning_Parameters.mpcobj;


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

[Y,U,yopt,uopt] = closedloop_toolbox_nmpc(nlobj,model,init,r,N,Nu,delta,lambda,nit);

% Plot simulation results
figure;
subplot(3,1,1);
plot(ts,r(1,:),'LineWidth',2);
hold on;
plot(ts,Y(1,:),'LineWidth',2);
ylabel('Concentration of B [mol/l]');
xlabel('Time [h]');
title('Closed-loop NMPC Simulation Results');
legend('Setpoint','Output','Location','best');
grid on;

subplot(3,1,2);
plot(ts,r(2,:),'LineWidth',2);
hold on;
plot(ts,Y(2,:),'LineWidth',2);
ylabel('Temperature [C]');
xlabel('Time [h]');
legend('Setpoint','Output','Location','best');
grid on;

subplot(3,1,3);
plot(ts,U(1,:),'LineWidth',2);
hold on;
plot(ts,U(2,:),'LineWidth',2);
ylabel('Control Inputs');
xlabel('Time [h]');
legend('Feed Flowrate','Heat Exchanged','Location','best');
grid on;
