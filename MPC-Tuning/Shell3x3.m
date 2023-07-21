%% Calculation of MPC control parameters for the Shell 3x3 Article
% This file configures GPC for the first case adapted to the Matlab toolbox,
% providing greater robustness and ease of code distribution.
% Modify the flags at the beginning of the code to set the desired
% configuration in the simulation.
%
% Giraldo, Sergio A. C., Príamo A. Melo, and Argimiro R. Secchi. 2022.
% "Tuning of Model Predictive Controllers Based on Hybrid Optimization"
% Processes 10, no. 2: 351. https://doi.org/10.3390/pr10020351
%
% Author:
% Sergio Andres Castaño Giraldo
% 2023
% https://controlautomaticoeducacion.com/
%

clc % Clears the command window
clear all % Clears all variables in the workspace
close all % Closes all figure windows
datetime % Displays the current date and time

%% ********* Code Configuration Flags ************
%true: find the tuning parameters, false: use .mat file to load the tuning parameters.
tuning = true; 
rest = true; % false: Without constraints; true: With constraints
caso = 2; % 1: Case 1 (fast); 2: Case 2 (slow)
nominal = true; % true: Nominal case; false: Model error case
lineal = true; % Linear model used in MPCTuning

addpath('MPC_Tuning') % Adds the 'MPC_Tuning' folder to the Matlab path

deltak = 0.0; % Fixed value of uncertainty in the gain
deltaL = 0; % Fixed value of uncertainty in the delay

%% Model error of the Shell column (To use it in a real plant)
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

%% Reference Trajectory for the tuning algorithm
if caso == 1
    Pref=[tf(1,[5 1]),0,0;0,tf(1,[9 1]),0;0,0,tf(1,[5.7 1])];
else
    Pref=[tf(1,[30 1]),0,0;0,tf(1,[30 1]),0;0,0,tf(1,[30 1])];
end
Pref.iodelay=[27,0,0;0,14,0;0,0,0];
Prefz=c2d(Pref,Ts,'zoh');

%% Minimum Plant Input-Output Delay
dmin(my,1)=100;
dreal=Pz.iodelay;
% Decomposes numerator, denominator and delay of the transfer function Pz in cells
[Bp,Ap,dp]=descompMPC(Pz);   
for i=1:my
    dmin(i)=min(dp(i,:));
end

%% Setpoint for the Shell example
inK=10;
Xsp(1,inK:80) = 0.2;Xsp(1,80:200) = 0;Xsp(1,200:300) = 0.1;Xsp(1,300:400) = 0.0;
Xsp(2,inK:80) = 0.2;Xsp(2,80:200) = 0.4;Xsp(2,200:300) = 0.3;Xsp(2,300:400) = 0.0;
Xsp(3,inK:80) = 0.2;Xsp(3,80:200) = 0.1;Xsp(3,200:300) = 0.0;Xsp(3,300:400) = 0.0;

%% Specify the MD vector
mdv = zeros(nit,0); %400×0 empty double matrix

% Reference response for compare in GAM algorithm
t = 0:Ts:(nit-1)*Ts;            % Time vector for simulation    
Yref = lsim(Pref,Xsp,t,'zoh');  % Simulate reference response using lsim function

%% create MPC controller object with sample time
% Create an MPC object with the specified sample time and input/output signal type using the 'setmpcsignals' function.
sysd = setmpcsignals(Pz,MV=[1;2;3]);

% Initialize the MPC object using the 'mpc' function.
% Specify the plant model, sample time, prediction and control horizons, and input/output signal type.
mpcverbosity off;
mpc_toolbox = mpc(sysd, Ts);

%% specify nominal values for inputs and outputs
% Specify nominal values for the plant inputs and outputs.
mpc_toolbox.Model.Nominal.U = [0;0;0];
mpc_toolbox.Model.Nominal.Y = [0;0;0];

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
    mpc_toolbox.MV(i).Min = Umn(i);
    mpc_toolbox.MV(i).Max = Umx(i);
    mpc_toolbox.MV(i).RateMin = InUmn(i);
    mpc_toolbox.MV(i).RateMax = InUmx(i);
end

%% specify constraints for OV
% Set constraints on the plant outputs.
for i = 1:my
    mpc_toolbox.OV(i).Min = Ymn(i);
    mpc_toolbox.OV(i).Max = Ymx(i);
end

%% specify simulation options
% Set simulation options for the MPC controller using the 'mpcsimopt' function.
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

%% MPC Tuning algorithm
if tuning == true
    w=[0.05 0.40 0.55]; %Pesos para la curva de pareto
    tic
        [mpc_toolbox,scale,delta,lambda,N,Nu,Fob] = MPCTuning(mpc_toolbox,Xsp,lineal,w,nit,Yref,mdv,7,4);
    toc
    L = scale.L;
    R = scale.R;
    Ru = scale.Ru;
    Rv = scale.Rv;
else
    [filename, pathname] = uigetfile('*.mat', 'Select a MAT-file with the MPC Tuning');
    if isequal(filename,0) || isequal(pathname,0)
       disp('User pressed cancel');
    else
       disp(['User selected ', fullfile(pathname, filename)]);
    end
    load(filename);
    N = Tuning_Parameters.N; Nu = Tuning_Parameters.Nu;
    delta = Tuning_Parameters.delta;
    lambda = Tuning_Parameters.lambda;
    L = Tuning_Parameters.scale.L;
    R = Tuning_Parameters.scale.R;
    Ru = Tuning_Parameters.scale.Ru;
    Rv = Tuning_Parameters.scale.Rv;
    mpc_toolbox = Tuning_Parameters.mpcobj;
end

%% Variables
ylab{1}='Top end point composition';
ylab{2}='Side end point composition';
ylab{3}='Bottoms reflux temperature';
ulab{1}='Top drawn flow rate';
ulab{2}='Side drawn flow rate';
ulab{3}='Bottoms reflux heat duty';

%% Open Loop Response
% the MPC, in a given Pareto front, finds an optimal trajectory (open loop)
% to be applied in the control law. However, as stated before, only the first 
% action is applied to the process because, in the next sampling time, there 
% will be updated process measures that allow the correction of the trajectory. 
% This is known as the receding horizon. However, if one only considers this
% first calculation, in $k=1$, and if the prediction and control horizons 
% are poorly selected, the prediction of this first trajectory will differ 
% significantly from the closed-loop behavior of the system when the receding 
% horizon concept is applied, then the predictions do not make sense because 
% the optimization does not represent what is really going to happen in the future.

% Verify if the prediction horizon and the control horizon were
% correctly selected, for that it is necessary to solve the MPC
% applying only the controller optimization once and taking all the
% calculated controls. Then we compare it with a conventional MPC
% controller and both responses should be very similar.

nit_open = N(1) + 30; % Number of iterations for the open loop simulation
t = 0:Ts:(nit_open-1)*Ts;
r_ma = ones(my, nit_open); % Reference for the model (step)
inK = 1; % Setpoint placed at instant 1

% Scale the signals using L and R matrices
r_ma = L*r_ma;
%mdv = R\mdv;

% Reference response with which tuning parameters were found
Yref_ma = lsim(Prefz, r_ma, t, 'zoh');

% Selector, to apply one setpoint at a time on each controlled variable
sel = zeros(my, 1);
for i = 1:ny
    sel(i) = 1; % Activate setpoint for controlled variable i
    try
        % Solve MPC in open loop
        [Xy1, Xu1, ~, Xyma1, Xuma1] = closedloop_toolbox(mpc_toolbox, r_ma.*sel,mdv, max(N), max(Nu), delta, lambda, nit_open);
        % Store responses in vectors for later plotting
        Xy(i, :) = Xy1(i, :); % Response of controlled variable of conventional MPC
        Xu(i, :) = Xu1(i, :); % Response of manipulated variable of conventional MPC
        Xyma(i, :) = Xyma1(i, :); % Response of controlled variable of MPC at instant k=1 (open loop)
        Xuma(i, :) = Xuma1(i, :); % Response of manipulated variable of MPC at instant k=1 (open loop)
    catch
        fprintf('Error in closed loop\n');
    end
    sel(i) = 0; % Reset selector to zero again
end
ch = max(max(Xuma', Xu')); % Variable used to plot length of horizons in graph
for i = 1:my
    figure
    hold on
    subplot(2, 1, 1)
    % Plot setpoint, closed loop, desired response (ref) and open loop
    plot(t, r_ma(i,:), t, Xy(i,:), t, Yref_ma(:, i), ':', t, Xyma(i,:), '--', 'Linewidth', 2)
    hold on
    plot(Ts*[inK+N+dmin(i) inK+N+dmin(i)], [0 r_ma(i, end)], 'Linewidth', 2)
    ylabel(ylab{i}, 'Interpreter', 'latex')
    
    subplot(2, 1, 2)
    stairs(t, Xu(i,:), 'Linewidth', 2)
    hold on
    plot(t(inK:inK+Nu(i)-1), Xuma(i, inK:inK+Nu(i)-1), '.k', 'MarkerSize', 25, 'Linewidth', 2)
    stairs([t(1:inK+Nu(i)-1), t(end)], [Xuma(i, 1:inK+Nu(i)-1), Xuma(i, inK+Nu(i)-1)], '--', 'Linewidth', 2)
    plot(Ts*[inK+Nu(i) inK+Nu(i)], [0 ch(i)], 'Linewidth', 2)
    ylabel(ulab{i}, 'Interpreter', 'latex')
end

%% specify prediction horizon
mpc_toolbox.PredictionHorizon = max(N);
%% specify control horizon
mpc_toolbox.ControlHorizon = max(Nu);
%% specify weights
mpc_toolbox.Weights.OV = delta;
mpc_toolbox.Weights.MVRate = lambda;
mpc_toolbox.Weights.ECR = 10000;


% Scale the signals using L and R matrices
r = row2col(L*Xsp);
% v = row2col(R\mdv);

%v = row2col(R\mdv);
real_plant = L * Psr * R;

%Define an actual plant model which differs from the predicted model
plant = setmpcsignals(real_plant,MV=[1;2;3]);

%Create and configure a simulation option set.
options = mpcsimopt(mpc_toolbox);
options.Model = plant;

% Simulate the controller.
[y,t,u,xp] = sim(mpc_toolbox,nit,r,[],options);

% Unscaled the vectors
y = row2col(L\y');
u = row2col(R*u');
r = row2col(L\r');


for i = 1:my
    figure(20)
    subplot(3,1,i); plot(t,u(:,i),'linewidth',2),grid;ylabel(['MV' num2str(i)])
    figure(21)
    subplot(3,1,i); plot(t,r(:,i),'--',t,y(:,i),t,Yref(:,i),':','linewidth',2),grid;ylabel(['MO' num2str(i)])
end

