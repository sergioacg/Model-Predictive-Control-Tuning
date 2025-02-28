%% Calculation of MPC control parameters for the FPSO example 
%  
%
% Author:
% Sergio Andres Castaño Giraldo
% 2025
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
nominal = true; % true: Nominal case; false: Model error case
lineal = true; % Linear model used in MPCTuning
simulink = true;

addpath('MPC_Tuning') % Adds the 'MPC_Tuning' folder to the Matlab path

N = 9; 
Nu = 3;
delta = [0  0  0  0  0  0  0  0  0  0  0  0  0  0];
lambda = [100000 100000 0.00001];
ECR = 1000;

%% Define the process
load('FPSO_Plant.mat')

Psr = H; %system 14x4, the last imput is the disturbance.
P_plant = Psr(:,1:3);
P_dis = Psr(:,end);

Ps =H;

% Sampling Period and Number of iterations
Ts=20.0;
nit=200;

% Number of Inputs and Outputs
my = 14; %controller variables
ny = 3; %manipulated variables
nv = 1; %disturbance

% Discrete-time Model
Pz=c2d(Ps,Ts,'zoh');

Km=dcgain(Pz); % Calculate DC gain of plant model
[L,R,S] = CondMin(Km); % Minimize condition number using CondMin function
Ru = R(1:ny,1:ny);
Rv = R(ny+1:end,ny+1:end);

Pze = L*Pz*R;

%% Set constraints on the plant outputs.
Ymn = -5*ones(1,14);
Ymx = 5*ones(1,14);

%% Set constraints on the control inputs and their increments.
Umx=[10 10 10]';      % maximum control output
Umn=-Umx;               % minimum control output
InUmx=[1 1 1];
InUmn = -InUmx;

%% Reference Trajectory for the tuning algorithm
gr = tf(1,[80 1]);
Pref=blkdiag(gr,gr,gr,gr,gr,gr,gr,gr,gr,gr,gr,gr,gr,gr);

dmin = zeros(1, my);
Kg = dcgain(Ps);
for i = 1:my
    nonzero_elements = Kg(i, Kg(i, :) ~= 0);
    dmin(i) = min(Ps.iodelay(i, Kg(i, :) ~= 0));
end

Pref.iodelay=diag(dmin);
Prefz=c2d(Pref,Ts,'zoh');

%% Setpoint for the Shell example into MPC Tuning algorthm
inK=10;
Xsp(1:my,1:nit) = 0; 

%% Specify the MD vector
tmd = 5; %entry time of the measured disturbance.
mdv = zeros(nv,nit);
mdv(:,tmd:nit/2) = -5;
mdv(:,nit/2:end) = 5;

%% Reference response for compare in GAM algorithm
t = 0:Ts:(nit-1)*Ts;            % Time vector for simulation   
Xref = zeros(my,nit);
% The reference trajectory with positive impulses is assumed because all
% model gains are positive.
for i = 1:my
    Xref(i,tmd:tmd+5) = 0.1;
end
Yref = lsim(Pref,Xref,t,'zoh')';  % Simulate reference response using lsim function
% % Yref = L*Yref;

%% Specify the MPC signal type for the plant input signals.
sysd = setmpcsignals(Pze,MV=[1;2;3],MD=4);

%% create MPC controller object with sample time
mpc_toolbox = mpc(sysd, Ts);
%% specify prediction horizon
mpc_toolbox.PredictionHorizon = N;
%% specify control horizon
mpc_toolbox.ControlHorizon = Nu;
%% specify nominal values for inputs and outputs
mpc_toolbox.Model.Nominal.U = [0;0;0;0];
mpc_toolbox.Model.Nominal.Y = [0;0;0;0;0;0;0;0;0;0;0;0;0;0];
%% specify constraints for MV and MV Rate
for i = 1:ny
    mpc_toolbox.MV(i).Min = R(i,i)\Umn(i);
    mpc_toolbox.MV(i).Max = R(i,i)\Umx(i);
    mpc_toolbox.MV(i).RateMin = R(i,i)\InUmn(i);
    mpc_toolbox.MV(i).RateMax = R(i,i)\InUmx(i);
end

%% specify constraints for OV
for i = 1:my
    mpc_toolbox.OV(i).Min = L(i,i)*Ymn(i);
    mpc_toolbox.OV(i).Max = L(i,i)*Ymx(i);
    % specify constraint softening for OV
    if i > 7
        mpc_toolbox.OV(i).MinECR = 0.5;
        mpc_toolbox.OV(i).MaxECR = 0.5;
    else
        mpc_toolbox.OV(i).MinECR = 0.1;
        mpc_toolbox.OV(i).MaxECR = 0.1;
    end
end

%% MPC Scales
% set input and output range
Urange = Umx-Umn;
Yrange = Ymx-Ymn;

% scale manipulated variables
for i = 1:ny
    mpc_toolbox.ManipulatedVariables(i).ScaleFactor = R(i,i)\Urange(i);
end
% scale outputs
for i = 1:my
    mpc_toolbox.OV(i).ScaleFactor = L(i,i)*Yrange(i);
end
% distrubance variables
 mpc_toolbox.DisturbanceVariables(1).ScaleFactor = Rv*5;

 
%% specify weights
mpc_toolbox.Weights.MV = [0 0 0];
mpc_toolbox.Weights.MVRate = lambda;
mpc_toolbox.Weights.OV = zeros(1, my);
mpc_toolbox.Weights.ECR = ECR;

%% specify simulation options
% Set simulation options for the MPC controller using the 'mpcsimopt' function.
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';


coloresG = [30, 112, 0]/255;
colorBand = [0.4940 0.1840 0.5560];
fontsize = 18;

%% Closed Loop Simulation with internal Model
ylab{1}='$y_1$';ylab{2}='$y_2$';ylab{3}='$y_3$';ylab{4}='$y_4$';ylab{5}='$y_5$';
ylab{6}='$y_6$';ylab{7}='$y_7$';
ylab{8}='$y_8$';ylab{9}='$y_9$';ylab{10}='$y_{10}$';ylab{11}='$y_{11}$';ylab{12}='$y_{12}$';
ylab{13}='$y_{13}$';ylab{14}='$y_{14}$';
yolab{1}='$y_{o_1}(k|1)$';yolab{2}='$y_{o_2}(k|1)$';yolab{3}='$y_{o_3}(k|1)$';
yolab{4}='$y_{o_4}(k|1)$';yolab{5}='$y_{o_5}(k|1)$';yolab{6}='$y_{o_6}(k|1)$';yolab{7}='$y_{o_7}(k|1)$';
yolab{8}='$y_{o_8}(k|1)$';yolab{9}='$y_{o_9}(k|1)$';yolab{10}='$y_{o_{10}}(k|1)$';
yolab{11}='$y_{o_{11}}(k|1)$';yolab{12}='$y_{o_{12}}(k|1)$';yolab{13}='$y_{o_{13}}(k|1)$';yolab{14}='$y_{o_{14}}(k|1)$';
ulab{1}='$u_1$';ulab{2}='$u_2$';ulab{3}='$u_3$';

% sysd = setmpcsignals(Pze,MV=[1;2;3],MD=4);
% mpc_toolbox.Model.Plant

 %% Resultad
FS=16;
colores=[30, 112, 0]/255;
coloresR=[178, 201, 35]/255;

% Scale the signals using L and R matrices
r = row2col(L*Xsp);
v = row2col(Rv\mdv);

% Define the actual plant model, which differs from the predicted model
real_plant = L * Psr * R;
plant = setmpcsignals(real_plant, MV=[1;2;3], MD=[4]);

% Create and configure a simulation options set
options = mpcsimopt(mpc_toolbox);

% Simulate the controller

% If a discrepancy exists in the states, try using the discrete version of the plant
options.Model = c2d(plant, Ts, 'zoh');  % Using the discrete-time plant
%
%fcost

Yref = row2col(Yref);
if simulink == false
    [y, t, u, xp] = sim(mpc_toolbox, nit, r, v);
    % Unscaled the vectors
    y = row2col(L\y');
    u=row2col(Ru*u');
else
    sim('FPSO')
    % Graficar la evolución de la función de costo
    figure;
    plot(1:nit, j_cost, 'LineWidth', 1.5);
    xlabel('Iteración');
    ylabel('Función de Costo');
    title('Evolución de la Función de Costo en MPC');
    grid on;
    
    % Mostrar si hubo problemas de factibilidad
    if any(status < 0)
        disp('Advertencia: Hubo problemas de factibilidad en algunas iteraciones.');
        disp('Códigos QP encontrados:');
        disp(unique(status));
    else
        disp('Todas las soluciones fueron factibles.');
    end

end
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
    plot(t,Yref(:,i),':k','linewidth',2)
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
    plot(t,Yref(:,i),':k','linewidth',2)
    hold on
    plot(t,y(:,i),'color',coloresG,'linewidth',4)
    hold on
    plot([t(1) t(end)],[Ymn(i) Ymn(i)],'--',[t(1) t(end)],[Ymx(i) Ymx(i)],'--','color',colorBand,'linewidth',2)
    grid;ylabel(['$$y_' num2str(i) '$$'],'interpreter','latex')
    set(gca,'FontSize',fontsize,'TickLabelInterpreter','latex' )
    xlabel('time [min]','interpreter','latex')
end
legend('Reference trajectory','Closed-loop response Interstage Pressure', 'Soft-constraints','interpreter','latex')

