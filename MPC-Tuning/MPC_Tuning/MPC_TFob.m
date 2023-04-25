function [N,Nu,lambda,delta,Fvns,Fvf] = MPC_TFob(Par)
% This function calculates the tuning parameters for an MPC controller using a hybrid combination of two optimization algorithms. The decision variables of the algorithm are x=[gamma,N,Nu,delta,lambda]. The variables N and Nu are strictly integer variables to be found by the VNS algorithm. The real delta and lambda variables are the diagonal elements of the weight matrices that will be solved by the GAM.
%
% Inputs:
%   Par - Structure containing parameters for the tuning algorithm
%
% Outputs:
%   N - Prediction horizon
%   Nu - Control horizon
%   lambda - Lambda weights
%   delta - Delta weights
%   Fvns - Objective function value for VNS algorithm
%   Fvf - Objective function value for GAM algorithm
%
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

global F

Fva=10e8;   % Initial VNS cost
Fvf=1e15;   % Initial GAM cost

% Load the parameters
nit=Par.nit; % number of iterations
Ts=Par.Ts; % Sampling time
Temp=0:Ts:(nit-1)*Ts; % Time vector for simulation
Par.Temp=Temp; % Store time vector in parameter structure
my=Par.ny;      % Number of outputs of MIMO system
ny=Par.nu;      % Number of inputs of MIMO system
nbp=Par.nbp; % Number of bits for prediction horizon
X=Par.Xv; % Integer variables in bits [N Nu]
nrm=Par.nrm; % Flag for normalized system

% Separate the bits of the prediction horizon from those of the control horizon
X1=X(1:nbp); % Prediction horizon bits
X2=X(nbp+1:end); % Control horizon bits

Xv1=X1; % Binary vector with final solution for prediction horizon

for i=1:ny % Loop through inputs
    Xv2(i,:)=X2; % Binary vector with final solution for control horizon
end
% Save in parameter structure
Par.Xv1=Xv1;
Par.Xv2=Xv2;
kk=1;hi=0;
    
while kk==1 % Loop until stop criterion is met

    %% GAM ALGORITHM
    % Search all parameters using a multi-objective optimization
    %% fgoalattain  
    options = Par.opContr; % Optimization options for fgoalattain function
    %options = optimoptions('fgoalattain','Display','iter');
    options.EqualityGoalCount = length(Par.w); % The number of objectives that should be as near as possible to the goals
    goal = 0.001*ones(1,my); % Goal vector
    
    R=@(x)GAM_fun(x,Par); % Objective function for GAM algorithm
    [XOt,~,attainfactor] = fgoalattain(R,Par.x0,goal,Par.w,[],[],[],[],Par.lb1,Par.ub1,[],options); % Call fgoalattain function to solve multi-objective optimization problem
    
    % The attainment factor indicates the level of goal achievement. 
    % A negative attainment factor indicates over-achievement, positive 
    % indicates under-achievement.
    
    if attainfactor < 0 % If over-achievement
        disp(['over-achievement (Logro Alcanzado)=',num2str(attainfactor)]);
    else % If under-achievement
        disp(['under-achievement (Logro NO Alcanzado)=',num2str(attainfactor)]);
    end
    
    %Suppose the user has set an initial weight to zero for any output variables (OV).
    %In that case, it is considered that he wishes to work by bands for that 
    %variable, which will depend on the weights of the associated equal concern
    %for relaxation (ECR) to each variable constraint.
    if Par.lineal == 1
        if any(Par.mpcobj.Weights.OV == 0)
            sft = find(Par.mpcobj.Weights.OV == 0);
            XOt(sft) = 0;
        end
    else
        if any(Par.mpcobj.Weights.OutputVariables == 0)
            sft = find(Par.mpcobj.Weights.OutputVariables == 0);
            XOt(sft) = 0;
        end
    end
    
    Par.x0 = XOt; % Update initial guess for optimization
    if nrm==0 % If not normalized system
        delta=abs(XOt(1:my));  % Reference weighting parameter
        lambda=abs(XOt(my+1:my+ny)); % Control weighting parameter
    else % If normalized system
        delta=abs(Par.delta);
        lambda=abs(XOt(1:my));
    end
    
    Fgam=round(sum(F),2); % Calculate GAM cost
    disp(['Fgam=',num2str(Fgam),'; Delta=[',num2str(delta),']; Lambda=[',num2str(lambda),']']);
    
    % Compare to the previous GAM cost 
    if Fgam>=Fvf   % If the GAM cost is higher, increase the stop criterion variable
        hi=hi+1;
    else
        Fvf=Fgam; % Update the previous cost
        % Save the new parameters
        Par.lambda=lambda;
        Par.delta=delta;
    end

    %% VNS Algorithm
    [N,Nu,Xv1,Xv2,Fvns] = VNS2(Par); % Call VNS2 function to find integer variables (prediction and control horizons)    
    % If the VNS cost is lower, save the new parameters
    if Fvns< Fva % If VNS cost is lower than previous cost
        Fva=Fvns; % Update previous cost
        Par.N=N; % Update prediction horizon
        Par.Nu=Nu; % Update control horizon
        Par.Xv1=Xv1; % Update binary vector for prediction horizon
        Par.Xv2=Xv2; % Update binary vector for control horizon
    end

    if  hi>0   % Stop criterion
        kk=2; % Set kk to 2 to exit loop
    end

end % End of while loop

Par.lambda=lambda; % Update lambda weights in parameter structure
Par.delta=delta; % Update delta weights in parameter structure    
    
disp(['Fvg=',num2str(Fvf)]); % Display GAM cost
disp(['Gamma:',num2str(XOt(1))]); % Display gamma value
N=Par.N; % Get prediction horizon from parameter structure
Nu=Par.Nu; % Get control horizon from parameter structure
lambda=Par.lambda; % Get lambda weights from parameter structure
delta=Par.delta; % Get delta weights from parameter structure

end