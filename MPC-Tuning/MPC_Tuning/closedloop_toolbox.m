function [y,u,t,ys,uopt] = closedloop_toolbox(mpc_toolbox,r,v,N,Nu,delta,lambda,nit)
% Calculates the closed-loop response of an MPC controller using the Matlab Toolbox.
%
% [y,u,t,ys,uopt] = closedloop_toolbox(mpc_toolbox,r,N,Nu,delta,lambda,nit)
%
% The function returns the plant response, y, the control input, u, the simulation time, t, 
% the open-loop response of the system when only a single prediction horizon is calculated 
% without the sliding horizon correction characteristic of MPC, ys, and the optimal control 
% input calculated at a single simulation step. uopt. Note: To obtain uopt, it is necessary 
% that there is some change in setpoint to stimulate the controller. In particular, this 
% algorithm takes the last position of the step vector, r, to calculate the optimal 
% predictions in a single prediction window.
%
% Input arguments:
% mpc_toolbox   -  MPC object from the Matlab toolbox
% r             -  Setpoint vector with size Nu x nit
% v             -  Disturbance vector with size Nd x nit
% N             -  Prediction horizon
% Nu            -  Control horizon
% delta         -  Weights for setpoint tracking
% lambda        -  Weights for control increments
% nit           -  Number of simulation iterations for the closed loop
%
% Author:
% Sergio Andres Castaño Giraldo
% 2023
% https://controlautomaticoeducacion.com/

mpcverbosity off;
%% Internal Model
dimen = size(mpc_toolbox);
my = dimen(2); %Number of output variables
ny = dimen(1); %Number of manipulated variables
Pz = mpc_toolbox.Model.Plant;
Ts = Pz.Ts;
mpc_toolbox.ControlHorizon = 1;%Initialized value to avoid warnings
%% Specify prediction horizon
mpc_toolbox.PredictionHorizon = max(N);
%% Specify control horizon
mpc_toolbox.ControlHorizon = max(Nu);
%% Specify weights
mpc_toolbox.Weights.OV = delta;
mpc_toolbox.Weights.MVRate = lambda;

%% Closed Loop
% Convert r to column vector
r = row2col(r);
v = row2col(v);
%% Run the MPC controller in closed loop for nit iterations using the setpoint vector r
[y,t,u] = sim(mpc_toolbox,nit,r,v); %Matlab function for control - loop
% %Manual control-loop
% % Initialize variables for storing results
% y = zeros(nit, my); % Outputs
% u = zeros(nit, ny); % Control actions
% xp = zeros(nit, my); % Process states
% 
% % Initialize internal MPC states
% xmpc = mpcstate(mpc_toolbox);
% 
% % Define the time vector
% t = 0:Ts:(nit-1)*Ts;
% 
% for k = 3:nit
%     % Update the process state using lsim to simulate one time step
%     uv = [u(1:k-1, :)'; v(1:k-1, :)'];
%     y_temp = lsim(Pz, uv, t(1:k-1), 'zoh');
%    
%     % Store the state and output of the process at the end of the time step
%     y(k, :) = y_temp(end, :);
% 
%     % Apply the MPC controller at time instant k
%     u(k, :) = mpcmove(mpc_toolbox, xmpc, y(k, :), r(k, :), v(k, :));
%     
%     % Open-loop response
%     % Calculate the optimal control input based on the current state xc, 
%     % the last output yo, and the last setpoint r
%     if k==3
%         [~,Info] = mpcmove(mpc_toolbox, xmpc, y(k, :), r(end, :), v(end, :));
%     end
%     
% end

%% Open-loop response
% Run the MPC controller in open loop for one iteration using the setpoint vector r
nit_open = 1;
[yo] = sim(mpc_toolbox,nit_open,r,v);
% Get the current state of the MPC controller
xc = mpcstate(mpc_toolbox);
% Calculate the optimal control input based on the current state xc, the last output yo, 
% and the last setpoint r
[~,Info]  = mpcmove(mpc_toolbox, xc, yo(end,:), r(end,:), v(end,:));

% Create the optimal control input vector with the same length as the closed-loop simulation
if length(Info.Uopt) > nit
    uopt = Info.Uopt(1:nit,:);
else
    uopt = [Info.Uopt;  Info.Uopt(end,:).*ones(length(y)-length(Info.Uopt),length(lambda))];
end
% Calculate the open-loop response of the plant to the optimal control input uopt
ys = lsim(Pz,[uopt v],t);

% Convert output vectors back to row vectors
y = col2row(y);
t = col2row(t);
u = col2row(u);
ys = col2row(ys);
uopt = col2row(uopt);

end
