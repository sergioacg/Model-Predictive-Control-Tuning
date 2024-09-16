function [y,u,yopt,uopt] = closedloop_toolbox_nmpc(nmpcobj,model,init,r,N,Nu,delta,lambda,nit)
% Function that computes the closed-loop response of an NMPC controller using Matlab's toolbox.
%
% [y,u,yopt,uopt] = closedloop_toolbox_nmpc(nmpcobj,model,init,r,N,Nu,delta,lambda,nit,integrator)
%
% The function returns the plant's response, y, the control action, u, the simulation time, t,
% the open-loop response of the system when only a single prediction horizon is calculated
% without the characteristic sliding horizon correction of the MPC, yopt, the optimal control
% calculated only at a single time step showing the steps of the controller to reach the
% objectives only if it had a single chance to optimize once, uopt. Note: in order to obtain uopt,
% it is necessary that there is some setpoint change to stimulate the controller, in particular,
% this algorithm takes the last position of the step vector, r, to calculate the optimal
% predictions in a single prediction window.
%
% input parameters:
% nmpcobj       -  NMPC object from the Matlab toolbox
% model         -  Phenomenological model of the plant expressed as an anonymous function @(t,x,u)model(t,x,u)
% init          -  Model initialization structure:
%                   init.x0: Initial condition of the model
%                   init.xc: Array with the output states of the model
%                   init.u0: Initial condition of the control action
%                   init.integrator: Model integration function @ode45, @ode15s, etc
% r             -  Setpoint vector with size Nu x nit
% N             -  Prediction horizon
% Nu            -  Control horizon
% delta         -  Setpoint tracking weighting
% lambda        -  Control increment weighting
% nit           -  Number of closed-loop simulation iterations
% 
%
% Author:
% Sergio Andres Castaño Giraldo
% 2023
% https://controlautomaticoeducacion.com/

x0 = init.x0;
xc = init.xc;
u0 = init.u0;
integrator = init.integrator;

Ts = nmpcobj.Ts;
nx = nmpcobj.Dimensions.NumberOfStates;
ny = nmpcobj.Dimensions.NumberOfOutputs;
nu = nmpcobj.Dimensions.NumberOfInputs;

%% Specify prediction horizon
nmpcobj.PredictionHorizon = max(N);

%% Specify control horizon
nmpcobj.ControlHorizon = max(Nu);

%% Specify weights
nmpcobj.Weights.OutputVariables = delta;
nmpcobj.Weights.ManipulatedVariablesRate = lambda;

%% Simulate closed-loop system using nonlinear MPC controller
% Preallocate state and input vectors
X = zeros(nx,nit);
Y = zeros(ny,nit);
U = zeros(nu,nit);

% Set initial conditions
X(:,1) = x0';
Y(:,1) = x0(xc)';
U(:,1) = u0';

for i = 2:nit
    % Compute new control action using nlmpcmove function
    U(:,i) = nlmpcmove(nmpcobj,X(:,i-1),U(:,i-1),r(:,i)');    
    % Simulate system using control action
    [~, x_next] = integrator(@(t,x) model(t, x, U(:,i)), [0 Ts], X(:,i-1));
    X(:,i) = x_next(end,:)';
    Y(:,i) = X(2:end,i);
end

%% Open loop response
% Calculates the optimal control for a sampling instant. It is necessary to have
% a setpoint change to obtain uopt!
[~,~,info] = nlmpcmove(nmpcobj,x0',u0',r(:,end)');
uopt = [info.MVopt;  info.MVopt(end,:).*ones(length(Y)-length(info.MVopt),nu)];
% Convert output vectors back to row vectors
uopt = col2row(uopt);
% Preallocate state and input vectors
Xop = zeros(nx,nit);
yopt = zeros(ny,nit);
% Set initial conditions
Xop(:,1) = x0';
yopt(:,1) = x0(xc)';
for i = 2:nit  
    % Simulate system using optimal control action
    [~, x_next] = integrator(@(t,x) model(t, x, uopt(:,i)), [0 Ts], Xop(:,i-1));
    Xop(:,i) = x_next(end,:)';
    yopt(:,i) = Xop(2:end,i);
end
% Store the output and input signals
y = Y;
u = U;
