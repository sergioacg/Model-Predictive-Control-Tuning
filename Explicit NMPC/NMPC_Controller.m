function [duOt] = NMPC_Controller(Par)
% NMPC Controller using Single-Shooting and Explicit Model
% This function performs optimization to compute control actions for a Nonlinear Model Predictive Control (NMPC) system.
% It uses a single-shooting strategy, where the entire prediction horizon is considered as a single segment for optimization,
% and an explicit model of the process to predict future system behaviors.

% Extract parameters from the passed structure
lb = Par.lb;       % Lower bounds of control variables
ub = Par.ub;       % Upper bounds of control variables
Nu = Par.Nu;       % Control horizon
u = Par.u;         % Previous control actions
k = Par.k;         % Current time step

% Prepare constraints for the control actions
aux = [];
for i = 1:length(Nu)
    aux = [aux; repmat(u(i, k-1), Nu(i), 1)];  % Replicate last control action for each control horizon
end

li = lb - aux;  % Calculate lower bounds for optimization
ls = ub - aux;  % Calculate upper bounds for optimization

% Initial condition for the optimization variables
u0 = zeros(sum(Nu), 1);

% Define the objective function to minimize
J = @(X) objectiveFunction(X, Par);  % Handle to objective function

% Optimize the control actions using fmincon
[duOt, ~, fminflag] = fmincon(J, u0, [], [], [], [], li, ls, [], Par.opContr);

% Check the status of the optimization
if exist('fminflag', 'var')
    switch fminflag
        case -2
            fprintf('No feasible point found.\n');
        case 0
            fprintf('Too many function evaluations or iterations.\n');
        case 1
            fprintf('Optimization converged.\n');
    end
end


function [fobj] = objectiveFunction(X, Par)
% Objective Function for NMPC Controller
% This function evaluates the cost associated with control decisions based on predicted outputs, reference tracking, and control effort.

% Reallocate variables within the function
r = Par.r; % Reference trajectory
Qd = Par.Qd; % Weighting for reference tracking
Ql = Par.Ql; % Weighting for control effort increment
k = Par.k;   % Current sampling instance
N = Par.N;   % Prediction horizon
Nu = Par.Nu; % Control horizon
u = Par.u;   % Past control action
Ts = Par.Ts; % Sampling period
my = Par.my; % Number of output variables
ny = Par.ny; % Number of control variables
x_control = Par.x_control; % Indices of controlled states

% Initialize output and setpoint variables
Xy = zeros(my*N,1);  % Vector storing predicted outputs
Xyr = zeros(my*N,1); % Vector storing model deviations
Xsp = zeros(my*N,1); % Vector storing setpoints

% Setup setpoints
Xsp(1:N,1) = r(1,k);
for i = 1:my-1
    Xsp(i*N+1:(i+1)*N) = r(i+1,k);
end

% Control Effort Calculation
deltaU = X(1:Nu(1));
for i = 1:ny-1
    deltaU = [deltaU; X(sum(Nu(1:i))+1:sum(Nu(1:i+1)))];
end

% System predictions based on the controller model
% Y = f(Yp,u,up) + n
% Y = Predicted output
% Yp = Current and past outputs
% u = Future controls
% up = Past controls
% n = Disturbance - Difference between plant and model
xini = Par.x0plant;  % Initial condition for the controller
for i = 1:N
    % Compute the controlled variable values
    if i <= max(Nu)
        for j = 1:ny
            if i <= Nu(j)
                uf(j) = u(j,k-1) + deltaU(sum(Nu(1:j-1)) + i);
            end
        end
    end
    
    % Calculate predicted system output
    time = [0 Ts];  % Integrates over one sampling period
    [~, xplant] = ode23t(@(t, x) plant_model(t, x, uf), time, xini);
    
    % Update plant's initial condition
    xini = xplant(end,:);
    for j = 1:my
        Xy((j-1)*N+i) = xini(x_control(j));
    end
end

% Model Prediction Correction (n)
% Calculate the current predicted output using the real plant control action with the new initial condition of the plant at the current time.
% This shows how the controller's internal model behaves with that input and then calculates the difference to see the deviation from the real system.
% Note that if the real system had some deviation due to a disturbance, the calculation of the current predicted output will be quite different 
% since in the controller's model we are not adding or subtracting disturbance models, and based on that, the controller will have to correct that deviation.

time = [0 Ts];
[~, xplant] = ode23t(@(t, x) plant_model(t, x, u(:,k-1)), time, Par.x0plant);
xini = xplant(end,:)';
Xyk = xini(x_control);
Xplant = Par.x0plant(x_control);

% Calculate model deviation
Xyr(1:N,1) = Xplant(1) - Xyk(1);
for i = 1:my-1
    Xyr(i*N+1:(i+1)*N) = Xplant(i+1) - Xyk(i+1);
end

% Correct the model by adding the deviation to the predicted output
Xyc = Xy + Xyr;

% Calculate tracking error
error = Xsp - Xyc;

% Sum of squared errors
J1 = error' * Qd * error;

% Sum of control increments
J2 = deltaU' * Ql * deltaU;

% Total objective function for predictive control
fobj = J1 + J2;


