function [g h]=GAM_fun(X,Par)
% This function calculates the objective function value for the GAM algorithm. 
% It simulates the closed-loop response of the system using the current 
% weights and compares it to the reference trajectory to calculate the error.
%
% Inputs:
%   X - Vector of decision variables (weights)
%   Par - Structure containing parameters for the tuning algorithm
%
% Outputs:
%   g - Objective function value
%   h - Equality constraints (empty)
% by: Sergio Andres Castaño Giraldo
%    Federal University of Rio de Janeiro
%    LADES - Laboratory of Software Development
%    https://controlautomaticoeducacion.com/
% Reference:
%
% Article (Reference):
% Giraldo, Sergio A. C., Príamo A. Melo, and Argimiro R. Secchi. 2022.
% "Tuning of Model Predictive Controllers Based on Hybrid Optimization"
% Processes 10, no. 2: 351. https://doi.org/10.3390/pr10020351


global F % Global variable to store objective function value

%% Realocate variables
N=Par.N;        % Prediction horizon
Nu=Par.Nu;      % Control horizon
ny=Par.ny;      % Number of outputs
nu=Par.nu;      % Number of inputs
nd=Par.nd;     % Number of inputs disturbances
mdv=Par.mdv;     % inputs disturbances
Yref=Par.Yref;  % Reference response
nit=Par.nit;    % Tuning horizon
Xsp=Par.Xsp;    % Set-point
nrm=Par.nrm; % Flag for normalized system
mpcobj = Par.mpcobj; % MPC object

lineal = Par.lineal; % Flag for linear system
if lineal ~= 1 % If nonlinear system
    init = Par.init;   % Initial parameter to integrate the internal model
    model = Par.Pz;    % EDO of the process model
end

%% Decision variables (Try to find the weights.)
if nrm==0 % If not normalized system
    delta=abs(X(1:ny));   % Reference weighting parameter
    lambda=abs(X(ny+1:ny+nu)); % Control weighting parameter
    ECR = Par.ECR;                  % Weights.ECR
    % Suppose the user has set an initial weight to zero for any output variables (OV).
    % In that case, it is considered that he wishes to work by bands for that 
    % variable, which will depend on the weights of the associated equal concern
    % for relaxation (ECR) to each variable constraint.
    if lineal == 1
        if any(mpcobj.Weights.OV == 0) %by bands
            sft = find(mpcobj.Weights.OV == 0);
            delta(sft) = 0;
            ECR    = abs(X(end))*Par.ECR;  % Rewrite Weights.ECR (include as decision variable)
        end
    else
        if any(mpcobj.Weights.OutputVariables == 0)
            sft = find(mpcobj.Weights.OutputVariables == 0);
            delta(sft) = 0;
            ECR    = abs(X(end))*Par.ECR;                  % Weights.ECR
        end
    end
else % If normalized system
    delta=abs(Par.delta);
    lambda=abs(X(1:ny));   % Control weighting parameter
    ECR    = Par.ECR;      % Weights.ECR
end

%% Closed-loop simulation with internal model
if lineal ==1 % If linear system
    try
        [Xy] = closedloop_toolbox(mpcobj,Xsp,mdv,N,Nu,delta,lambda, ECR,nit); % Simulate closed-loop response using closedloop_toolbox function
    catch
        fprintf('Error in closed-loop simulation\n');
        Xy = ones(ny,nit)*inf;
    end
else % If nonlinear system
    try
        Xy = closedloop_toolbox_nmpc(mpcobj,model,init,Xsp,N,Nu,delta,lambda,nit); % Simulate closed-loop response using closedloop_toolbox_nmpc function
    catch
        fprintf('Error in closed-loop simulation\n');
        Xy = ones(ny,nit)*inf;
    end
end

% Assuming that the Yref Reference trajectory has the correct direction at 
% the time of the tuning project, in the case of working in bands and since 
% it is difficult to know the direction of the system when there are fewer 
% degrees of freedom, what we are trying to do is that Yref has at least the 
% exact direction of the output of the MPC controller calculated with the 
% current tuning parameters at the present instant of the optimization algorithm.
% if any(delta == 0)
%     for i=1:ny
%         if delta(i) == 0
%             if abs(max(Xy(i,1:end/2))) < abs(min(Xy(i,1:end/2)))
%                 Yref(i,:) = -1*Yref(i,:);
%             end
%         end
%     end
% end



%% Objective function value
if any(delta == 0) % If working with bands
    % Initialize upper and lower bands for each output variable (OV)
    band_upper = zeros(Par.ny, 1); % Upper band for each output
    band_lower = zeros(Par.ny, 1); % Lower band for each output

    % Extract bands from MPC object
    for i = 1:Par.ny
        band_upper(i) = Par.mpcobj.OV(i).Max; % Maximum constraint for output i
        band_lower(i) = Par.mpcobj.OV(i).Min; % Minimum constraint for output i
    end

    % Check if the closed-loop response violates the bands in steady state
    violations_upper = max(0, Xy - band_upper); % Violations above the upper band
    violations_lower = max(0, band_lower - Xy); % Violations below the lower band

    % Calculate penalty for violations
    J_band = sum(violations_upper'.^2 + violations_lower'.^2)'; % Squared penalty for violations    
    F = J_band; % Penalize based on band violations
else % If working with reference trajectory
    % Error between closed-loop response and reference trajectory
    errFA=Xy-Yref; % Calculate error between closed-loop response and reference trajectory
    %error_weigth = alpha.*errFA;
    %J1=(diag(error_weigth*error_weigth')); % Calculate squared error
    
    J1=(diag(errFA*errFA')); % Calculate squared error
    F = J1; % Penalize based on squared error
end

g=F; % Set output g to objective function value
h=[]; % No equality constraints

end % End of function.