function [mpcobj, scale, delta, lambda, Np, Nun, Fob] = MPCTuning(mpcobj_proj, Sp, linear, w, fi, Yref, varargin)
% MPCTuning: Find the optimal parameters for (N)MPC algorithm
%
% [mpcobj, scale, delta, lambda, Np, Nun] = MPCTuning(mpcobj_proj, Sp, linear, w, fi, Pref) 
% finds the optimal values of the MPC algorithm parameters. 
%
% The output variables are:
%
%   - mpcobj: MPC object from MPC Toolbox with the tuning found and the 
%             internally scaled model and constraints.
%             The user must scale the input and output signals of the MPC 
%             controller for correct calculations in the implementation.
%   - scale: struct containing L and R matrices for scaling the MPC signals
%   - delta: tracking setpoint weight
%   - lambda: control increment weight
%   - Np: prediction horizon
%   - Nun: control horizon
%
% Input Arguments:
%
% - mpcobj_proj: MPC object from MPC toolbox
% - Sp: internal setpoint 
% - linear: a boolean variable indicating if a linear system is used (true) 
%           or a non-linear system is used (false)
% - w: the weight of Pareto optimization
% - fi: iteration number
% - Yref: Desired reference trajectory for the optimization algorithm
%
% Optional input arguments for non-linear systems:
%
% [mpcobj, scale, delta, lambda, Np, Nun] = MPCTuning(mpcobj_proj, Sp, linear, w, fi, Yref, mdv)
%    uses `mdv` as the stimulus of the MPC disturbance, it can be used 
%    mainly when working with an MPC control by bands where there is NOT a 
%    fixed setpoint in some controlled variables. 
%    (default: mdv = 1×0 empty double row vector)
%
% [mpcobj, scale, delta, lambda, Np, Nun] = MPCTuning(mpcobj_proj, Sp, linear, w, fi, Yref, mdv, nbp)
%    uses `nbp` as the number of bits for the prediction horizon 
%    (default: 8 bits = 255 as a maximum prediction)
%
% [mpcobj, scale, delta, lambda, Np, Nun] = MPCTuning(mpcobj_proj, Sp, linear, w, fi, Yref, mdv, nbp, nbc)
%    uses `nbc` as the number of bits for the control horizon 
%    (default: 4 bits = 15 as a maximum prediction)
%
% [mpcobj, scale, delta, lambda, Np, Nun] = MPCTuning(mpcobj_proj, Sp, linear, w, fi, Yref, mdv, nbp, nbc, nlmodel, init)
%    sets `nlmodel` as the non-linear model of the process to be integrated
%    with ODE, and `init` as its initial condition.
%   `init` is a struct with the following fields:
%       - init.x0: Initial condition of the model
%       - init.xc: Array with the output states of the model
%       - init.u0: Initial condition of the control action
%       - init.integrator: Model integration function (@ode45, @ode15s, etc.)
%
% [mpcobj, scale, delta, lambda, Np, Nun] = MPCTuning(mpcobj_proj, Sp, linear, w, fi, Yref, mdv, nbp, nbc, nlmodel, init, optimset) 
%    sets the options of optimization for fgoalattain
%
% Output:
%
% - Fob: final objective value of the optimization process
%
% Author: Sergio Andres Castaño Giraldo
%         Federal University of Rio de Janeiro
%         https://controlautomaticoeducacion.com/
%
% Versions:
%
% - ver 1.0 - 2017
% - ver 1.1 - 2018
% - ver 1.2 - 2019 (07/08)
% - ver 1.3 - 2020 (05/07)
% - ver 1.4 - 2023 (30/03)
% - ver 1.5 - 2023 (30/04)
%
% Reference:
%
% Article (Reference):
% Giraldo, Sergio A. C., Príamo A. Melo, and Argimiro R. Secchi. 2022.
% "Tuning of Model Predictive Controllers Based on Hybrid Optimization"
% Processes 10, no. 2: 351. https://doi.org/10.3390/pr10020351
%

global Fv 

mpcobj = mpcobj_proj;
ni = nargin; % Number of input arguments

% Options for Gam Optimizer
options = optimoptions('fgoalattain','Display','iter',...
        'UseParallel',false,'DiffMinChange',0.5,...
        'StepTolerance',0.01,'FunctionTolerance',0.001,...
        'MaxIterations',400);

if linear == 1
    Pz = mpcobj.Model.Plant; % Get plant model from MPC object
    % Sizes of MIMO System
    dimen = size(mpcobj);
    my = dimen(2); %Number of output variables
    ny = dimen(1); %Number of manipulated variables
    %Number of disturbance variables
    dy = length(mpcobj.DisturbanceVariable);
    Ts=Pz.Ts; % Sampling time
else %nonlinear
    % Sizes of MIMO System
    my = length(mpcobj.OutputVariables); %Number of output variables
    ny = length(mpcobj.ManipulatedVariables); %Number of manipulated variables
    %Number of disturbance variables
    dy = length(mpcobj.MeasuredDisturbances);
end


% Check number of input arguments and assign values accordingly
if ni ==0+6
    mdv = zeros(dy,0); %disturbance variables input
    nbp=8;nbc=4;
 elseif ni == 1+6
    mdv=varargin{1};
    nbp=8; nbc=4;
 elseif ni == 2+6
    mdv=varargin{1};
    nbp=varargin{2}; nbc=4;
 elseif ni == 3+6
    mdv=varargin{1};
    nbp=varargin{2};nbc=varargin{3};
 elseif ni == 4+6
    error('When you place the nonlinear model, you need to append the initial condition of the model.')
 elseif ni == 5+6
    mdv=varargin{1};
    nbp=varargin{2};nbc=varargin{3};
    model=varargin{4};init=varargin{5};
 elseif ni == 6+6
    mdv=varargin{1};
    nbp=varargin{2};nbc=varargin{3};
    model=varargin{4};init=varargin{5};
    options  = varargin{6}; % Update options with user input
end
 
%Validation bits of Horizon and control prediction
if nbp<nbc
    nbp=nbc;
end

%Validation measured disturbance in the model
if dy == 0
    mdv = zeros(1,0);
end

% Normalized variable system
nrm=0; % Set normalized variable system to 0

Xsp=Sp; % Setpoint
% Load the model
if linear == 1 % If linear model
     % Scale the system model minimizing the condition number
     Km=dcgain(Pz); % Calculate DC gain of plant model
     [L,R,S] = CondMin(Km); % Minimize condition number using CondMin function
     Ru = R(1:ny,1:ny);
     Rv = R(ny+1:end,ny+1:end);
     scale.L = L;
     scale.R = R;
     scale.Ru = Ru;
     scale.Rv = Rv;
     Pze=L*Pz*R; % Scale plant model
     q0 = mpcobj.Weights.OV ;   % Weight tracking reference Initial value
     w0 = mpcobj.Weights.MVRate; % Weight control effort Initial value
    % Split the transfer function in numerator, denominator, and delays
    [~,~,dp]=descompMPC(Pze); 
    
    sysd = setmpcsignals(Pze,MV=1:ny,MD=ny+1:ny+dy);
    mpcobj.Model.Plant=sysd;
    for i = 1:ny
        mpcobj.MV(i).Min = R(i,i)\mpcobj.MV(i).Min;
        mpcobj.MV(i).Max = R(i,i)\mpcobj.MV(i).Max;
        mpcobj.MV(i).RateMin = R(i,i)\mpcobj.MV(i).RateMin;
        mpcobj.MV(i).RateMax = R(i,i)\mpcobj.MV(i).RateMax;
    end
    for i = 1:my
        mpcobj.OV(i).Min = L(i,i)*mpcobj.OV(i).Min;
        mpcobj.OV(i).Max = L(i,i)*mpcobj.OV(i).Max;
    end
    mpcobj.Model.Nominal.Y = L*mpcobj.Model.Nominal.Y;
    mpcobj.Model.Nominal.U = R\mpcobj.Model.Nominal.U;
    Yref = L*col2row(Yref);
    Xsp  = L*col2row(Xsp);
    try
        mdv = R(ny+1:end,ny+1:end)\mdv;
    catch
        
    end

else % If nonlinear model
    Ts = mpcobj.Ts; % Sampling time
    my = mpcobj.Dimensions.NumberOfOutputs; % Number of outputs
    ny = mpcobj.Dimensions.NumberOfInputs; % Number of inputs
    X0 = init.x0; % Initial state of model
    XC = init.xc; % Index of controlled states
    q0 = mpcobj.Weights.OutputVariables;   % Weight tracking reference Initial value
    w0 = mpcobj.Weights.ManipulatedVariablesRate;      % Weight control effort Initial value
    scale = []; 

%     It remains to implement the normalized states and variables in the 
%     algorithm adapted for the matlab toolbox.
%     xmax=Pz.xmax;
%     xmin=Pz.xmin;
%     umax=Pz.umax;
%     umin=Pz.umin;

%     % SetPoint
%     Xspref = Sp-row2col(X0(XC)); 
%  
%     % This dynamic does not necessarily need to be linear, I am doing it
%     % linear for convenience only.
%     t = 0:Ts:(fi-1)*Ts;  % Time vector for simulation 
%     Yref=lsim(Pref,Xspref,t,'zoh')+ones(fi,my).*X0(XC); % Simulate reference response using lsim function
%     
end
% Find the minimal delay of MIMO system
if linear == 1 % If linear model
    dmin(1:my)=100; % Initialize minimum delay vector
    for i=1:my % Loop through outputs
        dmin(i)=min(dp(i,:)); % Find minimum delay for each output
    end
end

% Initialization of Weights
delta(1:my)=1; % Initialize delta weights to 1
lambda(1:ny)=1; % Initialize lambda weights to 1

% Integer Variables for prediction horizon (N) and Control Horizon (Nu)
% Number of bits for integer variables
Fc=[]; % Weights of the bits
% Prediction horizon (N)
for i=nbp-1:-1:0 % Loop through prediction horizon bits
    Fc=[Fc 2^i]; % Calculate weight for each bit
end
% Control Horizon (Nu)
for i=nbc-1:-1:0 % Loop through control horizon bits
    Fc=[Fc 2^i]; % Calculate weight for each bit
end

% Vector of variable integer decision: The prediction and control horizon
% concatenated in Xv vector in binary
% Xv=[Prediction-Horizon  Control-Horizon]
Hp=2^nbp-1; % Maximum value for prediction horizon
Hc=2^nbc-1; % Maximum value for control horizon
Xv=[flip(de2bi(Hp,nbp)) flip(de2bi(Hc,nbc))]'; % Convert horizons to binary and concatenate

% Initial horizon
N(1:my)=Hp; % Set initial prediction horizon to maximum value
Nu(1:ny)=2; % Set initial control horizon to 2

% Initial past cost
Fv=1e30; % Set initial past cost to large value

% Find the Utopia Solution
fo(1:my)=0; % Initialize Utopia solution vector

% Initial Conditions

%% fgoalattain
x0=[ones(1,my).*q0 ones(1,ny).*w0]; % Initial guess for optimization
% Constraints
lb1=[ones(1,my)*1e-5 ones(1,ny)*1e-5]; % Lower bound for optimization variables

ub1=[]; % No upper bound for optimization variables

% Parameters
Par.ny=my;          % Number of outputs
Par.nu=ny;          % Number of inputs
Par.nd=dy;          % Number of disturbances
Par.nrm=nrm;        % Flag - Normalized system?
if linear ==true % If linear model
    Par.Pz=Pz;         % Discrete transfer function
    Par.dmin=dmin;      % Minimal delay of MIMO system
else % If nonlinear model
    Par.Pz=model;       % Nonlinear model
    Par.init = init;    % Initial condition for the model
end

%Par.Pref=Pref;      % Reference transfer function
Par.Yref=col2row(Yref);     % Reference trajectory response
Par.Xsp=Xsp;        % set-point vector
Par.mdv=mdv;        % disturbance vector
Par.N=N;            % Prediction horizon 
Par.Nu=Nu;          % Control horizon
Par.delta=delta;    % Reference tracking weight
Par.lambda=lambda;  % Reference tracking control action weight
Par.nit=fi;         % Predefined tuning horizon
Par.Fo=fo;          % Utopia point
Par.w=w;            % Weighting vector of GAM algorithm
Par.Xv=Xv;          % Integer variables in bits [N Nu]
Par.Fc=Fc;          % Weights of the bits
Par.nbp=nbp;        % Number of bits for prediction horizon
Par.nbc=nbc;        % Number of bits for control horizon
Par.x0=x0;          % Initial conditions for optimization method
Par.lb1=lb1;        % Lower bound constraint for optimization variables
Par.ub1=ub1;        % Upper bound constraint for optimization variables
Par.opContr=options;% Optimization options for fgoalattain function
Par.lineal = linear;% 1 = linear system, 0 = nonlinear system
Par.Ts = Ts;        % Sampling time
Par.mpcobj = mpcobj;% MPC object

%% Tuning Algorithm
[N,Nu,lambda,delta,Fvns,Fgam] = MPC_TFob(Par); % Call MPC_TFob function to tune MPC

%% specify prediction horizon
mpcobj.PredictionHorizon = max(N);
%% specify control horizon
mpcobj.ControlHorizon = max(Nu);
%% specify weights
if linear == 1
    mpcobj.Weights.OV = delta;
    mpcobj.Weights.MVRate = lambda;
    mpcobj.Weights.ECR = 10000;
else
    mpcobj.Weights.OutputVariables = delta;
    mpcobj.Weights.ManipulatedVariablesRate = lambda;
end

% Results
Np=max(N); % Prediction horizon
Nun=Nu; % Control horizon
Fob=[Fvns;Fgam']; % Objective function values
disp(['N=',num2str(N),';']); % Display prediction horizon
disp(['Nu=',num2str(Nu),';']); % Display control horizon
disp(['delta=[',num2str(delta),'];']); % Display delta weights
disp(['lambda=[',num2str(lambda),'];']); % Display lambda weights
disp(['Fob=[Fvns;Fgam]=[',num2str(Fob'),'];']); % Display objective function values
 
 % Get the name of the main script
S = dbstack(); % Get call stack information
callerName = S(2).name; % Get name of calling script

Tuning_Parameters.mpcobj = mpcobj;
Tuning_Parameters.N = Np; % Store prediction horizon in structure
Tuning_Parameters.Nu = Nun; % Store control horizon in structure
Tuning_Parameters.delta = delta; % Store delta weights in structure
Tuning_Parameters.lambda = lambda; % Store lambda weights in structure
Tuning_Parameters.scale = scale; %scale matrices
Tuning_Parameters.date = datetime; % Store current date and time in structure
save([callerName,'_Tuning_', datestr(datetime,'ddmmmyyyy_HH_MM')], 'Tuning_Parameters') % Save tuning parameters to file
