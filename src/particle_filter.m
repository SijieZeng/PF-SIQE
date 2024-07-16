% Clear workspace, command window, and close all figures
clear;
clc;
close all;
% Increase graphics timeout
set(0, 'DefaultFigureCreateFcn', @(fig, ~)set(fig, 'CloseRequestFcn', @(src, ~)close(src)));
%% State transition Parameters

% Simulation parameters
params.dt = 1;  % time step (s)
params.num_iterations = 100; % Number of time steps
params.num_vehicles = 10; % Number of vehicles 

% generate_traffic_signal_states
params.red_time = 20;   % Red light duration (s)
params.yellow_time = 3.5; % Yellow light duration (s)
params.green_time = 20; % Green light duration (s)
params.d_stop_line = 300; % Stop line position (m)

% decision making
params.D_h = 150;       % Decision distance in meters
params.T_reaction = 1.0;% Reaction time in seconds
params.p_stop = 0.5;    % Probability of stopping in the dilemma zone
params.b_max = 3.0; 
params.d_0 = 0; 

% traffic light decision model
params.T_discrete = 1.0; 
params.a = 1.0;
params.D_stop = 1;  % Assuming 1 represents D_stop
params.D_go = 2;  % Assuming 2 represents D_go
params.D_undecided = 3;

% IDM parameters
params.v_desired = 15;  % desired speed (m/s)
params.a_max = 1.0;     % maximum acceleration (m/s^2)
params.b = 1.5;         % comfortable deceleration (m/s^2)
params.s0 = 2.0;        % minimum gap (m)
params.T = 1.0;         % safe time headway (s)
params.delta = 4;       % acceleration exponent

% Acceleration noise
params.n_a = 0.1;       % Bound for uniform distribution U[-params.n_a, params.n_a]
%% Measurement Parameters

% loop detectors
params.d_loop1 = 200;
params.d_loop2 = 200;
params.d_loop3 = 200;
params.dt_loop1 = 1; % s
params.dt_loop2 = 1; % s
params.dt_loop3 = 1; % s
params.accuracy_loop2 = 0.9;
%params.actual_presence = 1;

% Floating sensors
params.sigma_GPS = 5;
%%
% particle filter
params.num_particles = 1000;
%%
disp('Contents of params:');
disp(params);
%%
% Initialize StateTransition structure
StateTransition = state_transition();

% Generate traffic signal states
S = StateTransition.generate_traffic_signal_states(params);
% Plot traffic light state as a horizontal line at y = d_stop_line
figure(1);
hold on;

d_stop_line = params.d_stop_line;
num_iterations = params.num_iterations;

for k = 1:num_iterations
    if S(k) == "red"
        stairs([k k+1], [d_stop_line d_stop_line], 'r', 'LineWidth', 2);
    elseif S(k) == "yellow"
        stairs([k k+1], [d_stop_line d_stop_line], 'y', 'LineWidth', 2);
    elseif S(k) == "green"
        stairs([k k+1], [d_stop_line d_stop_line], 'g', 'LineWidth', 2);
    end
end

xlabel('Time Step');
ylabel('Position (m)');
title('Traffic Signal States Over Time');
ylim([0 d_stop_line + 50]);
hold off;
%%
% Initialize parameters
params = struct();

% Initialize state transition and measurement functions
StateTransition = state_transition();
Measurement = measurement();

% Particle filter functions
PF = particle_filter_functions();

% Number of particles
num_particles = params.num_particles;
state_dim = 5;

% Initialize particles
init_bounds = [0, params.d_stop_line; 0, params.v_desired; -params.a_max, params.a_max; 1, 3; 1, 1];
particles = PF.initialize_particles(num_particles, params.num_vehicles, state_dim, init_bounds);

% Simulate measurements (this would typically be obtained from sensors)
measurements.c_tilde = Measurement.measure_c(Measurement.count_loop(params, ...));
measurements.o_tilde = Measurement.measure_o(Measurement.presence_loop(params, ...), params);
measurements.v_avg_tilde = Measurement.measure_v_avg(Measurement.speed_loop(params, ...));
measurements.d_tilde = Measurement.measure_d( ..., params);

% Main loop for particle filter
for k = 1:params.num_iterations
    % Predict particles
    particles = PF.predict_particles(particles, params, StateTransition);
    
    % Update weights based on measurements
    weights = PF.update_weights(particles, measurements, params, Measurement);
    
    % Resample particles based on weights
    particles = PF.resample_particles(particles, weights);
    
    % Estimate the state
    estimated_state = PF.state_estimations(particles);
    
    % Estimate queue length
    queue_length = PF.queue_estimations(estimated_state, params);
    
    % Display or store results as needed
    disp(['Iteration ', num2str(k), ': Queue length = ', num2str(queue_length)]);
end