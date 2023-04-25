function dydt = vandevusse_model(t, x, u)
% vandevusse_model - Van de Vusse non-isothermal reactor ODE model.
%
% USAGE:
%   dydt = vandevusse_model(t, x, u)
%
% INPUTS:
%   t     - scalar, current time.
%   x     - vector, current state of the system.
%           x(1): ca - concentration of reactant A (mol/L).
%           x(2): cb - concentration of reactant B (mol/L).
%           x(3): T  - temperature (°C).
%   u     - vector, current values of the manipulated variables.
%           u(1): fov - feed flow rate (h^-1).
%           u(2): Tk  - coolant temperature (°C).
%
% OUTPUTS:
%   dydt  - vector, time derivative of the system states.
%           dydt(1): dca/dt - rate of change of concentration of A (mol/L.h).
%           dydt(2): dcb/dt - rate of change of concentration of B (mol/L.h).
%           dydt(3): dT/dt  - rate of change of temperature (°C/h).
%
% NOTES:
%   This function implements the Van de Vusse non-isothermal reactor ODE model,
%
% Range of manipulated variables:
% 50 L/h < F < 350 L/h
% -9000 kJ/h < Q < 0 kJ/h
% 
% Author:
%
% Sergio Andres Castaño Giraldo
% https://controlautomaticoeducacion.com/ 
%
%


% Model parameters:
k10 = 1.287e12; % h^-1
k20 = 1.287e12; % h^-1
k30 = 9.043e9;  % L/molA.h
E1 = -9758.3;    % K
E2 = -9758.3;    % K
E3 = -8560.0;    % K
deltaAB = -4.20; % kJ/molA
deltaBC = 11.00; % kJ/molB
deltaAD = 41.85; % kJ/molA
p = 0.9342;      % kg/L
cp = 3.01;       % kJ/kg.K
Kw = 4032;       % kJ/h.K.m^2
Ar = 0.215;      % m^2
V = 10;          % L
T0 = 130.00;     % °C
Ca0 = 5.10;      % molA/L

% Extract manipulated variables:
fov = u(1); % h^-1
Tk = u(2);  % °C

% Extract state variables:
ca = x(1); % mol/L
cb = x(2); % mol/L
T = x(3);  % °C

% Kinetic constants
k1 = k10*exp(E1./(T+273.15));
k2 = k20*exp(E2./(T+273.15));
k3 = k30*exp(E3./(T+273.15));

%
%  Modeling equations
%
dydt = zeros(3,1);
dydt(1) = fov*(Ca0 - ca) -k1*ca - k3*ca*ca;
dydt(2) = -fov*cb + k1*ca - k2*cb;
dydt(3) = (1/(p*cp))*(k1*ca*deltaAB + k2*cb*deltaBC + k3*ca^2*deltaAD)...
    + fov*(T0-T) + (Kw*Ar/(p*cp*V))*(Tk-T);

