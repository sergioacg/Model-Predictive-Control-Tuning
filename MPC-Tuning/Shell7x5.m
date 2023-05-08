%% Calculation of MPC control parameters for the Shell 7x5 
%
%
% The following script demonstrates the utilization of the hybrid tuning 
% algorithm for an MPC controller applied to the Shell heavy oil fractionator 
% benchmark. This innovative approach for model predictive control tuning 
% takes into account nonsquare systems, with more outputs than inputs, as well 
% as set points or ranges for controlled variables. This methodology can be 
% employed for any predictive control algorithm, as it offers adjustable 
% parameters based on the implementation.
%
% Cite as:
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
tuning = false; 
rest = true; % false: Without constraints; true: With constraints
nominal = true; % true: Nominal case; false: Model error case
lineal = true; % Linear model used in MPCTuning

addpath('MPC_Tuning') % Adds the 'MPC_Tuning' folder to the Matlab path

%% Model error of the Shell column (To use it in a real plant)
if nominal == true
    e1 = 0.0; e2 = 0.0; e3 = 0.0; e4 = 0.0; e5 = 0.0; % Nominal case
else
    e1 = 0.2; e2 = 0.2; e3 = 0.3; e4 = 0.5; e5 = 0.5; % Model error case
end

%% Define the process
% Transfer function
Gr=[tf(4.05+2.11*e1,[50 1]) tf(1.77+0.39*e2,[60 1]) tf(5.88+0.59*e3,[50 1]);...
    tf(5.39+3.29*e1,[50 1]) tf(5.72+0.57*e2,[60 1]) tf(6.9+0.89*e3,[40 1]);...
    tf(3.66+2.29*e1,[9 1])  tf(1.65+0.35*e2,[30 1]) tf(5.53+0.67*e3,[40 1]);...
    tf(5.92+2.34*e1,[12 1]) tf(2.54+0.24*e2,[27 1]) tf(8.10+0.32*e3,[20 1]);...
    tf(4.13+1.71*e1,[8 1])  tf(2.38+0.93*e2,[19 1]) tf(6.23+0.30*e3,[10 1]);...
    tf(4.06+2.39*e1,[13 1]) tf(4.18+0.35*e2,[33 1]) tf(6.53+0.72*e3,[9 1]);...
    tf(4.38+3.11*e1,[33 1]) tf(4.42+0.73*e2,[44 1]) tf(7.2+1.33*e3,[19 1]);];
Gr.iodelay=[27 28 27;18 14 15;2 20 2;11 12 2;5 7 2;8 4 1;20 22 0];

% Disturbios
Dr=[tf(1.20+0.12*e4,[45 1]) tf(1.44+0.16*e5,[40 1]);...
    tf(1.52+0.13*e4,[25 1]) tf(1.83+0.13*e5,[20 1]);...
    tf(1.16+0.08*e4,[11 1]) tf(1.27+0.08*e5,[6 1]);...
    tf(1.73+0.02*e4,[5 1])  tf(1.79+0.04*e5,[19 1]);...
    tf(1.31+0.03*e4,[2 1])  tf(1.26+0.02*e5,[22 1]);...
    tf(1.19+0.08*e4,[19 1]) tf(1.17+0.01*e5,[24 1]);...
    tf(1.14+0.18*e4,[24 1]) tf(1.26+0.10*e5,[32 1]);];
Dr.iodelay=[27 27;15 15;0 0;0 0;0 0;0 0;0 0];

Psr=[Gr Dr];

%% Define el modelo
% Transfer Function Model (continuous-time)
Gs=[tf(4.05,[50 1]) tf(1.77,[60 1]) tf(5.88,[50 1]);...
    tf(5.39,[50 1]) tf(5.72,[60 1]) tf(6.9,[40 1]);...
    tf(3.66,[9 1]) tf(1.65,[30 1]) tf(5.53,[40 1]);...
    tf(5.92,[12 1]) tf(2.54,[27 1]) tf(8.10,[20 1]);...
    tf(4.13,[8 1]) tf(2.38,[19 1]) tf(6.23,[10 1]);...
    tf(4.06,[13 1]) tf(4.18,[33 1]) tf(6.53,[9 1]);...
    tf(4.38,[33 1]) tf(4.42,[44 1]) tf(7.2,[19 1]);];
Gs.iodelay=[27 28 27;18 14 15;2 20 2;11 12 2;5 7 2;8 4 1;20 22 0];

% Disturbios
Ds=[tf(1.20,[45 1]) tf(1.44,[40 1]);...
    tf(1.52,[25 1]) tf(1.83,[20 1]);...
    tf(1.16,[11 1]) tf(1.27,[6 1]);...
    tf(1.73,[5 1]) tf(1.79,[19 1]);...
    tf(1.31,[2 1]) tf(1.26,[22 1]);...
    tf(1.19,[19 1]) tf(1.17,[24 1]);...
    tf(1.14,[24 1]) tf(1.26,[32 1]);];
Ds.iodelay=[27 27;15 15;0 0;0 0;0 0;0 0;0 0];

Ps=[Gs Ds];

% Sampling Period and Number of iterations
Ts=4.0;
nit=200;

% Discrete-time Model
Pz=c2d(Ps,Ts,'zoh');

% Number of Inputs and Outputs
my = 7;
ny = 3;

%% Set constraints on the plant outputs.
Ymn =[-0.005, -0.005, -0.5, -0.5, -0.5, -0.5, -0.5]';
Ymx =[0.005, 0.005, 0.5, 0.5, 0.5, 0.5, 0.5]';

%% Set constraints on the control inputs and their increments.
Umx=[0.5 0.5 0.5]';      % maximum control output
Umn=-Umx;               % minimum control output

%% Reference Trajectory for the tuning algorithm
gr = tf(1,[50 1]);
Pref=blkdiag(gr,gr,gr,gr,gr,gr,gr);

Pref.iodelay=diag(min(Ps.iodelay'));
Prefz=c2d(Pref,Ts,'zoh');

%% Setpoint for the Shell example into MPC Tuning algorthm
inK=10;
Xsp(1:my,1:nit) = 0; 

%% Specify the MD vector
tmd = 20; %entry time of the measured disturbance.
mdv = zeros(2,nit);
mdv(:,20:end) = 0.5;

%% Reference response for compare in GAM algorithm
t = 0:Ts:(nit-1)*Ts;            % Time vector for simulation   
Xref = zeros(my,nit);
% The reference trajectory with positive impulses is assumed because all
% model gains are positive.
for i = 1:my
    Xref(i,tmd:tmd+5) = Ymx(i);
end
Yref = lsim(Pref,Xref,t,'zoh')';  % Simulate reference response using lsim function
% % Yref = L*Yref;

%% Specify the MPC signal type for the plant input signals.
sysd = setmpcsignals(Pz,MV=[1;2;3],MD=[4;5]);

%% create MPC controller object with sample time
mpc_toolbox = mpc(sysd, Ts);
%% specify prediction horizon
mpc_toolbox.PredictionHorizon = 20;
%% specify control horizon
mpc_toolbox.ControlHorizon = 8;
%% specify nominal values for inputs and outputs
mpc_toolbox.Model.Nominal.U = [0;0;0;0;0];
mpc_toolbox.Model.Nominal.Y = [0;0;0;0;0;0;0];
%% specify constraints for MV and MV Rate
for i = 1:ny
    mpc_toolbox.MV(i).Min = Umn(i);
    mpc_toolbox.MV(i).Max = Umx(i);
end

%% specify constraints for OV
for i = 1:my
    mpc_toolbox.OV(i).Min = Ymn(i);
    mpc_toolbox.OV(i).Max = Ymx(i);
    % specify constraint softening for OV
    mpc_toolbox.OV(i).MinECR = 1;
    mpc_toolbox.OV(i).MaxECR = 1;
end
mpc_toolbox.OV(1).MinECR = 0.1;
mpc_toolbox.OV(1).MaxECR = 0.1;
mpc_toolbox.OV(2).MinECR = 0.1;
mpc_toolbox.OV(2).MaxECR = 0.1;
%% specify weights
mpc_toolbox.Weights.MV = [0 0 0];
mpc_toolbox.Weights.MVRate = [0.1 0.1 0.1];
mpc_toolbox.Weights.OV = [0 0 0 0 0 0 0];
mpc_toolbox.Weights.ECR = 10000;

%% specify simulation options
% Set simulation options for the MPC controller using the 'mpcsimopt' function.
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

%% MPC Tuning algorithm
if tuning == true
    %w=[0.0001 0.0001 0.5 0.5 0.5 0.5 0.5]; %Pesos para la curva de pareto
    w=[0.0001 0.0001 1 0.5 1 0.5 1];
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
    mpc_toolbox = Tuning_Parameters.mpcobj;
    L = Tuning_Parameters.scale.L;
    R = Tuning_Parameters.scale.R;
    Ru = Tuning_Parameters.scale.Ru;
    Rv = Tuning_Parameters.scale.Rv;
end


% Scale the signals using L and R matrices
r = row2col(L*Xsp);
v = row2col(Rv\mdv);
real_plant = L * Psr * R;

%Define an actual plant model which differs from the predicted model
plant = setmpcsignals(real_plant,MV=[1;2;3],MD=[4;5]);

%Create and configure a simulation option set.
options = mpcsimopt(mpc_toolbox);
options.Model = plant;

% Simulate the controller.
[y,t,u,xp] = sim(mpc_toolbox,nit,r,v,options);

% Unscaled the vectors
y = row2col(L\y');
u=row2col(Ru*u');

Gse = L*Gs*Ru;
Dse = L*Ds*Rv;

fontsize = 18;
figure
for i = 1:3
    subplot(3,1,i); 
    stairs(t,u(:,i),'linewidth',4),grid;
    ylabel(['$$u_' num2str(i) '$$'],'interpreter','latex')
    xlabel('time [min]','interpreter','latex')
    set(gca,'FontSize',fontsize,'TickLabelInterpreter','latex')
    hold on
    plot([t(1) t(end)],[Umn(i) Umn(i)],'-r',[t(1) t(end)],[Umx(i) Umx(i)],'-r','linewidth',2)
end
legend('Manipulated variables', 'Hard-constraints','interpreter','latex')

figure
for i = 1:7
    subplot(4,2,i); 
    plot(t,y(:,i),'-b','linewidth',4)
    hold on
    plot([t(1) t(end)],[Ymn(i) Ymn(i)],'-r',[t(1) t(end)],[Ymx(i) Ymx(i)],'-r','linewidth',2)
    grid;ylabel(['$$y_' num2str(i) '$$'],'interpreter','latex')
    set(gca,'FontSize',fontsize,'TickLabelInterpreter','latex' )
    xlabel('time [min]','interpreter','latex')
end
legend('Controlled variables', 'Soft-constraints','interpreter','latex')