function [yp] = OptimalPredictor2(Fr, Pz, Gz, u, y, k)
    % The optimal predictor (OP)
    % [yp] = OptimalPredictor(Fr, Pz, Gz, u, y, k)
    %
    % yp = Output predictor
    % Fr = Filtered Smith Predictor
    % Pz = Process model
    % Gz = Rapid model
    % u = Control Action
    % y = Output Process
    % k = Actual discrete time
    %
    % Optimal predictors are used in the context of model predictive control (MPC).
    % While Smith predictors are used to compensate for pure dead time, optimal
    % predictors are usually employed to predict the future behavior of the plant
    % in a multistep ahead receding horizon. Optimal predictors do not explicitly
    % appear in the resulting MPC structure, although it has been shown that the
    % MPC structure is equivalent to an optimal predictor plus a primary controller.
    %
    % By: Sergio Andres Castaño Giraldo
    % Rio de Janeiro 2018
    % https://controlautomaticoeducacion.com

    Ts = Fr.Ts;
    t = 0:Ts:(k-1)*Ts;

    % Saida do Modelo
    ypz = lsim(Pz, u(:, 1:k), t, 'zoh')';

    % Saida do Modelo Rapio
    ygz = lsim(Gz, u(:, 1:k), t, 'zoh')';

    % Erro de modelo
    eM = y(:, 1:k) - ypz;

    % Salida del Filtro Fr
    yfr = lsim(Fr, eM, t, 'zoh')';

    % Salida Predita
    yp = ygz + yfr;
end
