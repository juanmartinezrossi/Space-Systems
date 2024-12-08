% Escape Velocity Calculation Script
% Constants
G = 6.67430e-11;       % Gravitational constant in m^3/kg/s^2
M = 5.972e24;          % Mass of Earth in kg
R = 6.371e6;           % Radius of Earth in meters

% Escape velocity formula
v_escape = sqrt(2 * G * M / R);

% Display the result
fprintf('The escape velocity at the Earth''s surface is %.2f m/s.\n', v_escape);
