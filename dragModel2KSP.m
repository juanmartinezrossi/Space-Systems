% Constants
R = 600000;           % Kerbin's radius in meters
G = 6.674e-11;         % Gravitational constant
M = 5.29e22;          % Kerbin's mass in kg
rho0 = 1.225;          % Sea-level air density in kg/m^3
H = 5600;              % Scale height in meters
g0 = 9.8;             % Surface gravity in m/s^2
dt = 0.01;             % Time step in seconds
t_final = 100000;      % Extended total simulation time in seconds

% Initial conditions for KSP rocket
Cd = 0.026;              % Drag coefficient (assumption)
m_initial = 5109;   
A = 1.54;              % Cross-sectional area in m^2 (example)
Isp = 250;             % ISP of KSP Rocket (sea level)
v = 0;                 % Initial velocity
h = 0;                 % Initial altitude
mass = m_initial;      % Set initial mass
mass_flow_rate = 68.5;
v_escape = 3431;       % Kerbin's escape velocity at sea level in m/s
dry_mass = 3002;    
height_objective = 70000; %objective height in meters

% Payload for KSP rocket
payload_mass = 0; 

% Initialize arrays for plotting
time_array = 0:dt:t_final;
altitude_array = zeros(size(time_array));
velocity_array = zeros(size(time_array));
mass_array = zeros(size(time_array));
drag_array = zeros(size(time_array));

% Initialize delta-v accumulator
delta_v_total = 0;
delta_v_drag = 0;
idx = 1;
F_max_drag = 0;
h_max = 0;

% Time loop
for t = 0:dt:t_final
    % Update gravitational acceleration at current altitude
    g = G * M / (R + h)^2;

    % Update air density based on current altitude
    rho = rho0 * exp(-h / H);

    % Calculate drag force based on current velocity and altitude
    D = 0.5 * A * Cd * rho * v^2;
    if D>F_max_drag
        F_max_drag=D;
    end

    % Calculate total thrust force based on combined ISP, mass flow rate, and surface gravity
    if mass >= dry_mass + payload_mass
        F = mass_flow_rate * Isp * g0;
    else
        F = 0;
    end

    % Net force and acceleration
    F_net = F - D - mass * g;
    a = F_net / mass;

    % Update velocity and altitude
    v = v + a * dt;
    h = h + v * dt;

    if h>h_max
        h_max=h;
    end

    % Update mass due to fuel consumption
    if mass > dry_mass + payload_mass
        mass = mass - mass_flow_rate * dt;
    else
        mass = dry_mass + payload_mass-1;
    end

    % Store data for plotting
    altitude_array(idx) = h;
    velocity_array(idx) = v;
    mass_array(idx) = mass;
    drag_array(idx) = D;

    % Increment delta-v
    delta_v_total = delta_v_total + abs(F * dt / mass);
    % Calculate drag Delta-V loss
    delta_v_drag = delta_v_drag + (D * dt / mass);


    % Break the loop if the velocity reaches or exceeds the escape velocity
    if mass <= dry_mass + payload_mass && v <= 0 
        break;
    end
    
    % Increment index for data storage
    idx = idx + 1;
end

% Output the results
disp(['Final velocity: ', num2str(v), ' m/s']);
disp(['Max altitude: ', num2str(h), ' m']);
disp(['Final mass: ', num2str(mass), ' kg']);
disp(['Payload mass: ', num2str(payload_mass), ' kg']); % Display payload mass
disp(['Total Delta-V: ', num2str(delta_v_total), ' m/s']); % Display accumulated delta-v
disp(['Total Delta-V drag loss: ', num2str(delta_v_drag), ' m/s']); % Display accumulated delta-v drag loss
disp(['Max Drag Force: ', num2str(F_max_drag), ' N']);

% Plot results
time_array = time_array(1:idx); % Trim arrays to actual simulation time
altitude_array = altitude_array(1:idx);
velocity_array = velocity_array(1:idx);
mass_array = mass_array(1:idx);
drag_array = drag_array(1:idx);

figure;
subplot(4, 1, 1);
plot(time_array, altitude_array / 1000); % Altitude in km
xlabel('Time (s)');
ylabel('Altitude (km)');
title('Altitude over Time');

subplot(4, 1, 2);
plot(time_array, drag_array); % Drag force in Newton
xlabel('Time (s)');
ylabel('Drag (N)');
title('Drag over Time');

subplot(4, 1, 3);
plot(time_array, velocity_array);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity over Time');

subplot(4, 1, 4);
plot(time_array, mass_array / 1000); % Mass in tonnes
xlabel('Time (s)');
ylabel('Mass (tonnes)');
title('Mass over Time');

