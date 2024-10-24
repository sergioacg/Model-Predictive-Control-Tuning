function dydt = plant_model(~, x, u)
% Plant Model for NMPC Controller
% This script defines the dynamic model of a chemical process involving
% reactions among species A, B, and C, modeled with Arrhenius kinetics.
%
% Initial conditions for state variables (not used directly in this function):
% CA0 = 1.235;   % Initial concentration of A [mol/L]
% CB0 = 0.9;     % Initial concentration of B [mol/L]
% T0  = 134.14;  % Initial temperature [°C]
% Tc0 = 128.95;  % Initial coolant temperature [°C]
% Initial conditions for manipulated variables (not used directly):
% F0  = 18.83;   % Initial flow rate [L/h]
% Q0  = -4495.7; % Initial heat transfer rate [kJ/h]

% Range for manipulated variables:
% 50 L/h < F < 350 L/h
% -9000 kJ/h < Q < 0 kJ/h

% Kinetic and thermodynamic data
k10 = 1.287e12; % Pre-exponential factor for reaction 1 [1/h]
k20 = 1.287e12; % Pre-exponential factor for reaction 2 [1/h]
k30 = 9.043e9;  % Pre-exponential factor for reaction 3 [L/molA·h]
E1 = -9758.3;   % Activation energy for reaction 1 [K]
E2 = -9758.3;   % Activation energy for reaction 2 [K]
E3 = -8560.0;   % Activation energy for reaction 3 [K]
deltaAB = -4.20; % Heat of reaction AB [kJ/molA]
deltaBC = 11.00; % Heat of reaction BC [kJ/molB]
deltaAD = 41.85; % Heat of reaction AD [kJ/molA]
p = 0.9342;     % Density of fluid [Kg/L]
cp = 3.01;      % Specific heat capacity [Kj/Kg·K]
Kw = 4032;      % Heat transfer coefficient [Kj/h·K·m^2]
Ar = 0.215;     % Reactor surface area [m^2]
V = 10;         % Reactor volume [L]
T0 = 130.00;    % Reference temperature [°C]
Ca0 = 5.10;     % Reference concentration of A [molA/L]

% Manipulated variables
fov = u(1); % Flow rate override [h^-1]
Tk = u(2);  % Coolant temperature [°C]

% State variable notation
ca = x(1); % Concentration of A
cb = x(2); % Concentration of B
T = x(3);  % Temperature of the reactor

% Kinetic constants calculation using Arrhenius equation
k1 = k10 * exp(E1 / (T + 273.15));
k2 = k20 * exp(E2 / (T + 273.15));
k3 = k30 * exp(E3 / (T + 273.15));

% Modeling equations
dydt = zeros(3,1);
dydt(1) = fov * (Ca0 - ca) - k1 * ca - k3 * ca * ca;
dydt(2) = -fov * cb + k1 * ca - k2 * cb;
dydt(3) = (1 / (p * cp)) * (k1 * ca * deltaAB + k2 * cb * deltaBC + k3 * ca^2 * deltaAD) ...
          + fov * (T0 - T) + (Kw * Ar / (p * cp * V)) * (Tk - T);
