function [y, u] = ClosedLoopNMPC(x0_model, x_control, u0, r, N, Nu, Q, W, nit, ub1, lb1, inK, Ts)
    % Simulate a closed-loop operation of an NMPC controller
    %
    % [y, u] = ClosedLoopNMPC(x0_model, x_control, u0, r, N, Nu, delta, lambda, nit, ub1, lb1, inK, Ts)
    %
    % Outputs:
    %   y        - Closed-loop system output
    %   u        - Control action
    %
    % Inputs:
    %   x0_model - Steady-state of the system (all states)
    %   x_control - States to be controlled by the NMPC
    %   u0       - Initial control value (steady-state input)
    %   r        - System reference trajectory (r(1:ny,1:nit))
    %   N        - Prediction horizon
    %   Nu       - Control horizon (array)
    %   Q        - Weights for tracking setpoint
    %   W        - Weights for control effort
    %   nit      - Number of iterations
    %   ub1      - Upper limits of controlled states
    %   lb1      - Lower limits of controlled states
    %   inK      - Initial simulation step
    %   Ts       - Sampling time

    % Plant and controller size definitions
    my = length(Q);
    ny = length(W);

    % Assemble weighting matrices for the optimization problem
    Wm = [];
    for i = 1:ny
        Wm = blkdiag(Wm, W(i) * eye(Nu(i)));  % Weighting matrix for control effort
    end
    Wm = sparse(Wm);
    Qm = [];
    for i = 1:my
        Qm = blkdiag(Qm, Q(i) * eye(N(1)));  % Weighting matrix for setpoint tracking
    end
    Qm = sparse(Qm);

    % Initial conditions setup
    x0 = x0_model;  % Use steady-state model as initial condition

    % Ensure column vectors for state and control initial conditions
    if size(x0, 2) ~= 1
        x0 = x0';
    end
    if size(u0, 2) ~= 1
        u0 = u0';
    end

    x0plant = x0;  % Initial condition for the plant model
    % Prepare bounds for the optimization
    ub = [];
    lb = [];
    for i = 1:length(Nu)
        ub = [ub; ub1(i) * ones(Nu(i), 1)];
        lb = [lb; lb1(i) * ones(Nu(i), 1)];
    end

    % Control loop variables initialization
    u(1:ny, 1:nit) = repmat(u0, 1, nit);  % Control action initialization
    delU = zeros(ny, nit);  % Control increment initialization
    y(1:my, 1:nit) = repmat(x0(x_control), 1, nit);  % Output model process initialization

    % Optimization settings
    opContr = optimset('Algorithm', 'sqp', 'UseParallel', false, ...
        'Display', 'off', 'TolX', 1e-6, 'TolFun', 1e-7, ...
        'TolCon', 1e-5, 'MaxFunEvals', 8000);

    % Simulation values saved in a structure for optimization
    Par_NL = struct('x0', x0, 'r', r, 'Ql', Wm, 'Qd', Qm, ...
        'opContr', opContr, 'ub', ub, 'lb', lb, 'N', N, 'Nu', Nu,...
        'x0plant', x0plant, 'Ts', Ts, 'u', u, 'my', my, 'ny', ny,...
        'x_control', x_control);

    noise_magnitude = 0.01;  % Ajuste este valor según sea necesario

    % Control loop
    for k = inK:nit
        Par_NL.k=k;
        % Numerical integration of the plant model for one sample time
        time = [0 Ts];
        [~, xplant] = ode45(@(t, x) plant_model(t, x, u(:, k-1)), time, x0plant);

        % Update the plant condition  % Add noise to x0plant
        x0plant = xplant(end, :)'+ noise_magnitude * randn(size(xplant(end, :)')); 

        % Update the current plant response in the NMPC model controller
        Par_NL.x0plant = x0plant;
        y(:, k) = x0plant(x_control);

        % Compute NMPC control law
        duo = NMPC_Controller(Par_NL);

        %first row of each control horizon block
        delU(1, k) = duo(1);   
        for i = 1:ny - 1
            if sum(Nu(1:i)) + 1 <= length(duo)
                delU(i + 1, k) = duo(sum(Nu(1:i)) + 1);  
            end
        end

        % Update control signals
        u(:, k) = u(:, k - 1) + delU(:, k);

        % Store control actions in the optimization parameter structure
        Par_NL.u = u;
    end
end
