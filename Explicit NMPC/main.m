%% SERGIO ANDRRES CASTAÑO GIRALDO
% NMPC Control for Van der Vusse Reaction
% Federal University of Rio de Janeiro
% Rio de Janeiro - 2017
% www.controlautomaticoeducacion.com
% https://www.youtube.com/c/SergioACastañoGiraldo
% ======================================================
% This script sets up and runs a Nonlinear Model Predictive Control (NMPC) simulation for a MIMO system 
% implementing the Van der Vusse reaction. Parameters for simulation, NMPC tuning, system constraints, and 
% reference trajectories are defined. The script solves for the NMPC control actions over a specified simulation 
% period using the 'ClosedLoopNMPC' function, and plots the results.

close all
clear
clc
tic

%% MIMO System

%% Simulation Time Parameters
Ts=0.05;        % Sampling time
nit=150;         % Number of simulation iterations

%% Initial System Condition
% Steady-state of the system
CA0 = 5.1;              % Concentration of A [mol/L]
CB0 = 1.1163;           % Concentration of B [mol/L]
T0  = 130;              % Temperature [C]
x0 = [CA0 CB0 T0];

% Defining system parameters
u0 = [20 130];   

ts = 0:Ts:(nit-1)*Ts;   % Simulation time

options = optimset('Display','off');
% Steady State
x0 = fsolve(@(x)plant_model(ts,x,u0),x0,options);

%% System Constraints
% Control action limits
Cbupp = 1.2;
Cblow = 0;
Fupp  = 150;            % Max flow rate [L/h]
Flow  = 0;              % Min flow rate [L/h]
Qupp  = 150;            % Max heat exchanged [kJ/h]
Qlow  = 40;             % Min heat exchanged [kJ/h]

ub1=[Fupp Qupp];
lb1=[Flow Qlow];
inK=4;

%% Reference Trajectory
r(1,1:10)=x0(2);  r(1,10:nit)=1.2; r(1,50:nit)=1.0;
r(2,1:nit)=x0(3);  r(2,81:nit)=130; r(2,111:nit)=120;

%% NMPC Tuning Parameters
N=5;           % Prediction horizon
Nu=[2 2];       % Control horizon
Q=[1.0214    0.9999];       % Reference tracking
W=[1.0e-04    1.0e-04];    % Control effort

%% Control Loop
[y, u] = ClosedLoopNMPC(x0, [2 3], u0, r, N, Nu, Q, W, nit, ub1, lb1, inK, Ts);

%% Results
yCb=y(1,inK:end);
yT=y(2,inK:end);
r1=r(1,inK:end);
r2=r(2,inK:end);
u1=u(1,inK:end);
u2=u(2,inK:end);
ts = 0:Ts:(nit-1)*Ts;
ts=ts(1:length(yCb));

% Plotting Real Plant Results
ftsz1 = 24;  % Font size
ftsz2 = 24;  % Font size

figure
subplot(221)
plot(ts, yCb, '-', ts, r1, '--', 'LineWidth', 3);
xlabel('Time (h)', 'FontSize', ftsz2);
ylabel('C_b', 'FontSize', ftsz2);
title('Concentration of B [mol/L]', 'FontSize', ftsz2);
ylim([0.8 1.2]);
set(gca, 'FontSize', ftsz1);
% Displays the concentration of component B over time, comparing actual values to reference trajectories.

subplot(223)
stairs(ts, u1, 'k', 'LineWidth', 3);
xlabel('Time (h)', 'FontSize', ftsz2);
ylabel('F/V', 'FontSize', ftsz2);
title('Flow rate to volume ratio [h^{-1}]', 'FontSize', ftsz2);
set(gca, 'FontSize', ftsz1);
% Plots the flow rate to volume ratio as a controlled variable with constraints shown as red lines.

subplot(222)
plot(ts, yT, ts, r2, '--', 'LineWidth', 3);
xlabel('Time (h)', 'FontSize', ftsz2);
ylabel('T', 'FontSize', ftsz2);
title('Reactor Temperature [°C]', 'FontSize', ftsz2);
set(gca, 'FontSize', ftsz1);
% Depicts the temperature inside the reactor, showing both actual measurements and desired setpoints.

subplot(224)
stairs(ts, u2, 'k', 'LineWidth', 3);
hold on;
xlabel('Time (h)', 'FontSize', ftsz2);
ylabel('T_k', 'FontSize', ftsz2);
title('Controlled Temperature [°C]', 'FontSize', ftsz2);
set(gca, 'FontSize', ftsz1);
% Shows the controlled temperature setting within operational constraints, highlighted with red boundary lines.

toc
% Measures and displays the total computation time.

