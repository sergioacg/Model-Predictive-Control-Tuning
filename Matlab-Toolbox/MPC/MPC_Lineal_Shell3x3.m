%% Example of MPC Implementation for the Shell Heavy Oil Fractionator (HOF) 3x3 MIMO Subsystem
% This example demonstrates the implementation of a linear MPC controller 
% based on a 3x3 MIMO subsystem from the Shell Heavy Oil Fractionator (HOF), 
% a well-known reference system in process control literature. 
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
% - Curso de Controladores con PIC : http://bit.ly/Control_PIC
% - Curso de Controladores con Arduino: http://bit.ly/Control_Ardu
% - Applied Control System in Arduino: https://bit.ly/AppControlCAE
%________________________________________________________________

clc % Clears the command window
clear  % Clears all variables in the workspace
close all % Closes all figure windows
datetime % Displays the current date and time

%% ********* Code Configuration Flags ************
rest = true; % false: Without constraints; true: With constraints
nominal = true; % true: Nominal case; false: Model error case
simulink = false; %Simulate the process with Simulink (true) / Matlab (false)

%% Model error of the Shell column (To use it in a real plant)
deltak = 0.0; % Fixed value of uncertainty in the gain
deltaL = 0; % Fixed value of uncertainty in the delay

if nominal == true
    e1 = 0.0; e2 = 0.0; e3 = 0.0; % Nominal case
else
    e1 = 0.2; e2 = 0.2; e3 = 0.3; % Model error case
end

%% Define the process
% Transfer function
Psr = [tf(4.05+2.11*e1,[50 1]) tf(1.77+0.39*e2,[60 1]) tf(5.88+0.59*e3,[50 1]);...
tf(5.39+3.29*e1,[50 1]) tf(5.72+0.57*e2,[60 1]) tf(6.9+0.89*e3,[40 1]);...
tf(4.38+3.11*e1,[33 1]) tf(4.42+0.73*e2,[44 1]) tf(7.2+1.33*e3,[19 1]);];

Lr = [27 28 27;18 14 15;20 22 0]+0; % Delay
Psr.iodelay = Lr; % Adds the delay to the transfer function

%% Variables
ylab{1}='Top end point composition';
ylab{2}='Side end point composition';
ylab{3}='Bottoms reflux temperature';
ulab{1}='Top drawn flow rate';
ulab{2}='Side drawn flow rate';
ulab{3}='Bottoms reflux heat duty';

%% Define el modelo
% Transfer Function Model (continuous-time)
Ps=[tf(4.05,[50 1]) tf(1.77,[60 1]) tf(5.88,[50 1]);...
tf(5.39,[50 1]) tf(5.72,[60 1]) tf(6.9,[40 1]);...
tf(4.38,[33 1]) tf(4.42,[44 1]) tf(7.2,[19 1]);];

% Input/output delay matrix
Ls = [27 28 27;18 14 15;20 22 0];
Ps.iodelay=Ls;

% Sampling Period and Number of iterations
Ts=4.0;
nit=400;

% Model uncertainties
Ps=Ps*(1+deltak);
Ps.iodelay=Ls+deltaL;

% Discrete-time Model
Pz=c2d(Ps,Ts,'zoh');

% Number of Inputs and Outputs
[my,ny]=size(Pz);

% the minimum conditioning of the model
cond(dcgain(Pz))

%% Scale the system model minimizing the condition number
Km=dcgain(Pz); % Calculate DC gain of plant model
[L,R,S] = CondMin(Km); % Minimize condition number using CondMin function
Pze=L*Pz*R; % Scale plant model

%% create MPC controller object with sample time
% Create an MPC object with the specified sample time and input/output signal type using the 'setmpcsignals' function.
sysd = setmpcsignals(Pze,MV=[1;2;3]);

% Initialize the MPC object using the 'mpc' function.
% Specify the plant model, sample time, prediction and control horizons, and input/output signal type.
mpcverbosity off;
mpc_toolbox = mpc(sysd, Ts);

%% specify nominal values for inputs and outputs
% Specify nominal values for the plant inputs and outputs.
mpc_toolbox.Model.Nominal.U = L*[0;0;0];
mpc_toolbox.Model.Nominal.Y = R\[0;0;0];

%% Constraints
% Specify constraints for the MPC controller inputs and outputs.
% If the 'rest' variable is True, set hard constraints for the control inputs and soft constraints for the plant outputs.
% Otherwise, set no constraints for inputs or outputs.
if rest == true
    InUmx=[0.05 0.05 0.05]; % maximum control increment
    InUmn=-InUmx; % minimum control increment
    Umx=[0.5 0.5 0.5]; % maximum control output
    Umn=-[1 1 1]; % minimum control output
    Ymx=inf*ones(1,3); % maximum plant output
    Ymn=-inf*ones(1,3); % minimum plant output
else
    InUmx=inf*ones(mu,1);
    InUmn=-InUmx;
    Umx=inf*ones(mu,1);
    Umn=-Umx;
    Ymx=inf*ones(my,1);
    Ymn=-Ymx;
end

%% specify constraints for MV and MV Rate
% Set constraints on the control inputs and their increments.
for i = 1:ny
    mpc_toolbox.MV(i).Min = R(i,i)\Umn(i);
    mpc_toolbox.MV(i).Max = R(i,i)\Umx(i);
    mpc_toolbox.MV(i).RateMin = R(i,i)\InUmn(i);
    mpc_toolbox.MV(i).RateMax = R(i,i)\InUmx(i);
end

%% specify constraints for OV
% Set constraints on the plant outputs.
for i = 1:my
    mpc_toolbox.OV(i).Min = L(i,i)*Ymn(i);
    mpc_toolbox.OV(i).Max = L(i,i)*Ymx(i);
end

%% MPC Tuning algorithm
% Tuning Algorithm: 
% https://github.com/sergioacg/Model-Predictive-Control-Tuning/tree/main/MPC-Tuning
N = 24; 
Nu = [6 2 2];
delta = [0.0107    0.0040    0.0008];
lambda = [0.0001    0.0006    0.0015];
%% specify prediction horizon
mpc_toolbox.PredictionHorizon = max(N);
%% specify control horizon
mpc_toolbox.ControlHorizon = max(Nu);
%% specify weights
mpc_toolbox.Weights.OV = delta;
mpc_toolbox.Weights.MVRate = lambda;
mpc_toolbox.Weights.ECR = 10000;


%% specify simulation options
% Set simulation options for the MPC controller using the 'mpcsimopt' function.
options = mpcsimopt(mpc_toolbox);
% Disable look-ahead for reference signals during the simulation.
options.RefLookAhead = 'off';
% Disable look-ahead for measured disturbances during the simulation.
options.MDLookAhead = 'off';
% Enable enforcement of constraints during the simulation.
options.Constraints = 'on';
% Set the simulation to run in closed-loop mode (not open-loop).
options.OpenLoop = 'off';

%% Setpoint for the Shell example
inK=10;
Xsp(1,inK:80) = 0.2;Xsp(1,80:200) = 0;Xsp(1,200:300) = 0.1;Xsp(1,300:400) = 0.0;
Xsp(2,inK:80) = 0.2;Xsp(2,80:200) = 0.4;Xsp(2,200:300) = 0.3;Xsp(2,300:400) = 0.0;
Xsp(3,inK:80) = 0.2;Xsp(3,80:200) = 0.1;Xsp(3,200:300) = 0.0;Xsp(3,300:400) = 0.0;

% Scale the signals using L and R matrices
r = row2col(L*Xsp);
% v = row2col(R\mdv);

% Reference response for compare in GAM algorithm
t = 0:Ts:(nit-1)*Ts;            % Time vector for simulation    

% Simulate the controller.
if simulink == false
    [y,t,u,xp] = sim(mpc_toolbox,nit,r,[],options);
    % Unscaled the vectors
    y = row2col(L\y');
    u = row2col(R*u');
    r = row2col(L\r');
else
    sim('MPC_Shell3x3') 
end


for i = 1:my
    figure(20)
    set(gcf, 'Position', get(0, 'Screensize')); % Maximizar figura
    subplot(3,1,i); plot(t,u(:,i),'linewidth',2),grid;ylabel([ulab{i}])
    set(gca, 'FontSize', 14);
    figure(21)
    set(gcf, 'Position', get(0, 'Screensize')); % Maximizar figura
    subplot(3,1,i); plot(t,r(:,i),'--',t,y(:,i),'linewidth',2),grid;ylabel([ylab{i}])
    set(gca, 'FontSize', 14);
end


