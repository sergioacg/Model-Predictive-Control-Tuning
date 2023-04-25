function [N,Nu,Xv1,Xv2,Fvns] = VNS2(Par)
% Variable Neighborhood Search (VNS), proposed by Mladenovic and Hansen in 1997,
% is a metaheuristic method for solving a set of combinatorial optimization 
% and global optimization problems. It explores distant neighborhoods of the 
% current incumbent solution and moves to a new one if and only if an improvement 
% was made. The local search method is applied repeatedly to move from solutions 
% in the neighborhood to local optima. VNS was designed for approximating solutions 
% of discrete and continuous optimization problems, including linear programming, 
% integer programming, mixed-integer programming, nonlinear programming, etc.
%
% Inputs:
%   Par - Parameters for the VNS algorithm
%
% Outputs:
%   N - Prediction horizon of the MPC control
%   Nu - Control horizon of the MPC control
%   Xv1 - Binary vector representing the prediction horizon, which is varied with VNS
%   Xv2 - Binary vector representing the control horizon, which is varied with VNS
%   Fvns - Cost of the VNS optimization algorithm
%
% https://en.wikipedia.org/wiki/Variable_neighborhood_search
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


global Fv
% Load the Parameters
Fc = Par.Fc;    % cost function vector
nit = Par.nit;  % number of iterations
Yref = Par.Yref;    % reference signal
lambda = Par.lambda;    % weight of control horizon
delta = Par.delta;  % weight of control increments
nbp = Par.nbp;  % index of last binary digit
inK = 10;   % delay between input and output in time steps
Ts = Par.Ts;    % sampling time
Xv1 = Par.Xv1;  % current plant state
Xv2 = Par.Xv2;  % plant state after delay
my = Par.ny;    % number of outputs of MIMO system
ny = Par.nu;    % number of inputs of MIMO system

% Create time vector
Temp = 0:Ts:(nit-1)*Ts;

Xsp = Par.Xsp;  % reference for the model (step)

mpcobj = Par.mpcobj;
lineal = Par.lineal;  % Get the value of the "lineal" field from the "Par" structure
if lineal == 1
    % For linear systems, set the reference signal to a step function
    Xsp(1:my,1:nit) = 0;
    Xsp(1:my,inK:end) = 1;  
    Pref = Par.Pref;    % Get the plant transfer function from the "Par" structure
    Yref = lsim(Pref,Xsp,Temp,'zoh')';  % Compute the reference output signal using the plant transfer function
    dmin = Par.dmin;    % Get the minimal delay of the MIMO system from the "Par" structure
end

lineal = Par.lineal;  % Get the value of the "lineal" field from the "Par" structure again
if lineal ~= 1
    % For nonlinear systems, set the initial parameters and ODE of the model
    init = Par.init;    % Get the initial parameters of the model from the "Par" structure
    model = Par.Pz;     % Get the ODE of the model from the "Par" structure
    dmin = zeros(1,my); % Set the minimal delay of the MIMO system to zero
end

Iter = 0;   % Initialize the "Iter" variable to zero
Fc1 = Fc(1:nbp);    % Get the first "nbp" elements of the "Fc" array
Fc2 = Fc(nbp+1:end);    % Get the remaining elements of the "Fc" array

Nt1 = length(Fc1);  % Get the number of elements in the "Fc1" array
Nt2 = length(Fc2);  % Get the number of elements in the "Fc2" array

N  = Par.N;     % Get the control horizon from the "Par" structure
Nu = Par.Nu;    % Get the prediction horizon from the "Par" structure
Nu1 = Nu;

Par.Temp = Temp;    % Store the "Temp" variable in the "Par" structure
ii = 1;     % Initialize the "ii" variable to one

while ii<=3
    Ix=[];
    %Select the vns search order
    Order=ii;
    tt=1;
    m=1; %Actual output
    n=1; %Actual Input
    Hpc=my;
    H=1;
    while tt<=2
       while H<=Hpc
        if tt==1 %For Prediction Horizon
            Nt=Nt1;X3=Xv1;Fc3=Fc1;
        end
        if tt==2 %For Control Horizon
            Nt=Nt2;X3=Xv2(H,:)';Fc3=Fc2;
        end     

        %In the variable Ix, it saves the search indexes for each order.
        Ix(1)=Nt;
        for i=2:Order
            Ix(i)=Ix(i-1)-1;
        end


        while Ix(1)>=1 && tt~=0
            % In this FOR, the indices that are generally static are varied
            for t=1:Order-1
                if Ix(t)<=Nt && Ix(t)>0
                    if X3(Ix(t))==0,X3(Ix(t))=1;else,X3(Ix(t))=0;end
                end
            end
            %The WHILE varies the last index
            while Ix(end)>=1
                %Varies a bit
                if X3(Ix(end))==0,X3(Ix(end))=1;else,X3(Ix(end))=0;end

                %Transforms the bits in the MPC horizons
                if tt==1, N(1:my)=Fc3*(X3); else
                Nu(H)=Fc3*X3; end
                Nu1(H)=Fc2*Xv2(H,:)';
                
                %validates the horizons
                if ~PreCon(N,Nu) || ~PreCon(N,Nu1) || N(H)<=dmin(H) || Nu(H)<=1
                    if X3(Ix(end))==0
                        X3(Ix(end))=1;
                    else
                        X3(Ix(end))=0;
                    end
                    if tt==1
                        N(1:my)=Fc3*(X3); 
                    else                        
                        Nu(H)=Fc3*X3; 
                    end
                else
                    sel=zeros(my,1);
                    for i=1:my
                        sel(i)=1;
                        try
                            if lineal ==1
                                [Xy1,Xu1,~,Xyma1,Xuma1] = closedloop_toolbox(mpcobj,Xsp.*sel,max(N),max(Nu),delta,lambda,nit);
                            else
                                [Xy1,Xu1,Xyma1,Xuma1] = closedloop_toolbox_nmpc(mpcobj,model,init,Xsp.*sel,max(N),max(Nu),delta,lambda,nit);
                            end
                            Xy(i,:)=Xy1(i,:);
                            Xu(i,:)=Xu1(i,:);
                            Xyma(i,:)=Xyma1(i,:);
                            Xuma(i,:)=Xuma1(i,:);
                        catch
                            fprintf('Error in closed-loop simulation\n');
                        end
                        sel(i)=0;
                    end
                    
                    %% error
                    error2=Xy(:,inK:end) - Xyma(:,inK:end);
                    errYref=Xy(:,inK:end)-Yref(:,inK:end);

                    %Objectives                    
                    j21=(diag(error2*error2'));
                    j22=(diag(errYref*errYref'));

                    % Because as the control horizon increases, the control increments become 
                    % increasingly insignificant and become almost constant, I am trying to 
                    % determine if there was a significant change in relation to the previous 
                    % control increment.
                    dffXu = abs(diff(Xuma')');
                    %Xnu = (1./dffXu);
                    Xnu = (abs(Xuma(:,inK))./dffXu);
                    %Xnu = (dffXu(:,inK)./dffXu);
                    Xnu(isinf(Xnu)|isnan(Xnu)) = 0; % Replace NaNs and infinite values with zeros
                    Jnu = diag(Xnu*Xnu');
                    
                    %% Objective Function
                    F=sum(j21+j22)+N(1)+sum(Jnu); %Version Final paper
 
                    %Validation in VNS algorithm
                    if F < Fv %If it improved the goal
                        Fv=F; %Update the previous Cost
                        disp(['Fvns=',num2str(Fv),'; N=[',num2str(N),']; Nu=[',num2str(Nu),']']);
                        %Restart the search because it have a better solution.
                        Ix(1)=Nt;
                        for i=2:Order
                            Ix(i)=Ix(i-1)-1;
                        end
                        Ix(end)=-1;
                        Ix(1)=-1;
                        
                        %Código alternativo (Pruebas) Cambia N y Nu
                        XX=de2bi(N(1),length(Fc1));                              
                        Xv1=flip(XX)';
                        XX=de2bi(Nu,length(Fc2));  
                        for i=1:my
                            Xv2(i,:)=flip(XX(i,:));
                        end 
                        
                    else % IF IT DOESN'T IMPROVE THE OBJECTIVE FUNCTION
                        if X3(Ix(end))==0,X3(Ix(end))=1;else,X3(Ix(end))=0;end %Discard the neighbor
                        if tt==1, N(1:my)=Fc3*(X3); else
                        Nu(H)=Fc3*X3; end
                    end
                end                
                Ix(end)=Ix(end)-1; %Decrease counter        
            end

            if tt==1
                X3=Xv1;
            end
            if tt==2
                X3=Xv2(H,:)';
            end               

            if Order==1 && Ix(1)<0
                Ix(1)=Nt;
                pos=1;
            else
                %Initialize "pos" at NON NULL value and different from 1
                pos=2;
            end
            %Begin While
            while ~isempty(pos) && pos~=1
                %Search if any of the indices reached the end of the binary vector
                pos=find(Ix==0);
                % If it reached the end, and this is not index 1, it means that it is
                % time to update the penultimate index
                if ~isempty(pos) && pos~=1
                    Ix(pos-1)=Ix(pos-1)-1; %Incrementa el penultimo indice
                    Ix(pos)=Ix(pos-1)-1; %Reinicia el ultimo indice una posición encima del penultimo
                end
                pos=find(Ix==0); %Verifica nuevamente que no haya ningun indice en la ultima posición.
            end
            %En el caso que la función objetivo haya mejorado debo actualizar el
            %ultimo indice que fue modificado abrubtamente arriba con el fin de que
            %el algoritmo saliera del ciclo while.
            if isempty(pos) && Ix(1)~= -1
                for j=find(Ix<0, 1 ):Order
                    Ix(j)=Ix(j-1)-1;
                end
            end
           %Vuelva a la incumbente de la solución actual. 
           Iter=Iter+1;
        end

        if tt==0
            H=Hpc; %Busca de nuevo reiniciando el H
        end
        H=H+1;
       end
    H=1;
    if m<=1
        Hpc=my; 
    else
        Hpc=ny; 
    end
    tt=tt+1;
    end
    ii=ii+1;
end   

N(1:my)=Fc1*Xv1;

for i=1:ny
    Nu(i)=Fc2*Xv2(i,:)';
end

Fvns=Fv; %Retorna el valor del objetivo actual
end