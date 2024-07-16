% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% Increase graphics timeout
set(0, 'DefaultFigureCreateFcn', @(fig, ~)set(fig, 'CloseRequestFcn', @(src, ~)close(src)));

% Debug code for initialize_particles function
PF = particle_filter_functions();
ST = state_transition();
M = measurement();
% Set up test parameters
% State transition Parameters
params = struct();
% Simulation parameters
params.dt = 1;  % time step (s)
params.num_iterations = 10; % Number of time steps
params.num_vehicles = 1; % Number of vehicles 
params.num_particles = 5;

% generate_traffic_signal_states
params.red_time = 8;   % Red light duration (s)
params.yellow_time = 3.5; % Yellow light duration (s), cannot tune by now
params.green_time = 5; % Green light duration (s)
params.delta_t = 0.5;
params.d_stop_line = 300; % Stop line position (m)               
params.lane = 1;

% decision making
params.D_h = params.d_stop_line - 150;       % Decision distance in meters
params.T_reaction = 1.0;% Reaction time in seconds
params.p_stop = 0.5;    % Probability of stopping in the dilemma zone
params.b_max = 1.5; 
params.d_0 = 0; 

% traffic light decision model
params.T_discrete = 1.0; 
params.a = 1.0;
params.D_stop = 1;  % Assuming 1 represents D_stop
params.D_go = 2;  % Assuming 2 represents D_go
params.D_undecided = 3; % Assuming 2 represents D_undecided

% IDM parameters
params.v_desired = 15;  % desired speed (m/s)
params.a_max = 1.0;     % maximum acceleration (m/s^2)
params.b = 1.5;         % comfortable deceleration (m/s^2)
params.s0 = 2.0;        % minimum gap (m)
params.vehicle_length = 5.0; %vehicle length (m)
params.T = 1.0;         % safe time headway (s)
params.delta = 4;       % acceleration exponent

% Acceleration noise
params.n_a = 0.1;  

% Measurement Parameters
% loop detectors
params.d_loop1 = 200;
params.d_loop2 = 200;
params.d_loop3 = 200;
params.dt_loop1 = 1; % s
params.dt_loop2 = 1; % s
params.dt_loop3 = 1;
% s
params.accuracy_loop2 = 0.95;

% Floating sensors
params.sigma_GPS = 5;

%% generate state-based prediction output 

init_bounds = [0, params.D_h;
               params.v_desired - 1, params.v_desired;
               - params.b_max, params.a_max];

% Call the function
particles = PF.initialize_particles(init_bounds, params);

num_iterations = params.num_iterations;
num_particles = size(particles, 1);
num_vehicles = size(particles, 2);
state_dim = size(particles, 3);

% Initialize array to store predictions for all time steps
all_predictions = zeros(num_iterations, num_vehicles, state_dim);

% Calculate the mean state across all particles
initial_mean_state = squeeze(mean(particles, 1));
if num_vehicles == 1
    initial_mean_state = initial_mean_state(:)';  % Ensure it's a row vector
end

all_predictions(1, :, :) = initial_mean_state;

% Initialize state for each vehicle
%initial_states = struct('d', {}, 'v', {}, 'a', {}, 'D', {});
%for i = 1:params.num_vehicles
    %initial_states(i).d = initial_mean_state(i, 1);  % Position
    %initial_states(i).v = initial_mean_state(i, 2);  % Velocity
    %initial_states(i).a = initial_mean_state(i, 3);  % Acceleration
    %initial_states(i).D = initial_mean_state(i, 4);  % Decision
%end


% Print initial states
disp('Initial States:');
for i = 1:params.num_vehicles
    fprintf('Vehicle %d: d=%.2f, v=%.2f, a=%.2f, D=%d\n', ...
        i, initial_mean_state(i, 1), initial_mean_state(i, 2), ...
        initial_mean_state(i, 3), round(initial_mean_state(i, 4)));
end

% Predict for multiple time steps
for t = 2:num_iterations
    % Predict particles for this time step
    particles = PF.predict_particles(particles, params, ST);
    
    % Calculate mean state for this time step
    mean_state = squeeze(mean(particles, 1));
    
    % Store mean state for this time step
    all_predictions(t, :, :) = mean_state;
    
    % Print predicted states for this time step
    fprintf('\nPredicted States at time step %d:\n', t);
    for i = 1:num_vehicles
         fprintf('Vehicle %d: d=%.2f, v=%.2f, a=%.2f, D=%d\n', ...
            i, mean_state(i, 1), mean_state(i, 2), ...
            mean_state(i, 3), round(mean_state(i, 4)));
    end
end

% Optionally, you can plot the predictions over time
figure;
for i = 1:num_vehicles
    subplot(4, 1, 1);
    plot(1:num_iterations, squeeze(all_predictions(:, i, 1)));
    hold on;
    ylabel('Position (d)');
    title('Predicted States Over Time');
    
    subplot(4, 1, 2);
    plot(1:num_iterations, squeeze(all_predictions(:, i, 2)));
    hold on;
    ylabel('Velocity (v)');
    
    subplot(4, 1, 3);
    plot(1:num_iterations, squeeze(all_predictions(:, i, 3)));
    hold on;
    ylabel('Acceleration (a)');
    
    subplot(4, 1, 4);
    plot(1:num_iterations, squeeze(all_predictions(:, i, 4)));
    hold on;
    ylabel('Decision (D)');
    xlabel('Time Step');
end
%legend('Vehicle 1', 'Vehicle 2', ... , 'Vehicle N' ) ;


% Calculate mean predicted state
mean_predicted_state = squeeze(mean(predicted_particles, 1));
if params.num_vehicles == 1
    mean_predicted_state = reshape(mean_predicted_state, [params.num_vehicles, size(predicted_particles, 3)]);
end

% Output predicted states
disp('Predicted States:');
for i = 1:params.num_vehicles
    fprintf('Vehicle %d: d=%.2f, v=%.2f, a=%.2f, D=%d\n', ...
        i, mean_predicted_state(i, 1), mean_predicted_state(i, 2), ...
        mean_predicted_state(i, 3), round(mean_predicted_state(i, 4)));
end

% Calculate and display changes
disp('State Changes:');
for i = 1:params.num_vehicles
    delta_d = mean_predicted_state(i, 1) - initial_states(i).d;
    delta_v = mean_predicted_state(i, 2) - initial_states(i).v;
    delta_a = mean_predicted_state(i, 3) - initial_states(i).a;
    delta_D = round(mean_predicted_state(i, 4)) - initial_states(i).D;
    
    fprintf('Vehicle %d: Δd=%.2f, Δv=%.2f, Δa=%.2f, ΔD=%d\n', ...
        i, delta_d, delta_v, delta_a, delta_D);
end


%% 
S = ST.generate_traffic_signal_states(params);
% Prepare data for plotting
num_vehicles = params.num_vehicles;
num_iterations = params.num_iterations;
time_steps_vehicle = (0:params.num_iterations -1) * params.dt;
time_steps_signal = (0:length(S)-1) * params.delta_t;

% Initialize arrays to store states
positions = zeros(num_iterations, num_vehicles);
velocities = zeros(num_iterations, num_vehicles);
accelerations = zeros(num_iterations, num_vehicles);
decisions = zeros(num_iterations, num_vehicles);

% Extract data from all_states
for t = 1:num_iterations
    for i = 1:num_vehicles
        positions(t, i) = all_states{t}(i).d;
        velocities(t, i) = all_states{t}(i).v;
        accelerations(t, i) = all_states{t}(i).a;
        decisions(t, i) = all_states{t}(i).D;
    end
end

