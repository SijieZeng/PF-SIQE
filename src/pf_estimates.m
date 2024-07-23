% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% Increase graphics timeout
set(0, 'DefaultFigureCreateFcn', @(fig, ~)set(fig, 'CloseRequestFcn', @(src, ~)close(src)));

% Debug code for initialize_particles function
PF = particle_filter_functions();
ST = state_transition_functions();
M = measurement_functions();
% Set up test parameters
% State transition Parameters
params = struct();
% Simulation parameters
params.dt = 1;  % time step (s)
params.num_iterations = 40; % Number of time steps
params.num_vehicles = 3; % Number of vehicles 
params.num_particles = 100;

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
%% continue update weights, resampling, estimates

state_dim = 4;

% Initialize particles (reuse your existing initialization if available)
init_bounds = [0, params.D_h;
               params.v_desired - 1, params.v_desired;
               -params.b_max, params.a_max;
               ];  % Assuming decision is 1, 2, or 3
particles = PF.initialize_particles(init_bounds, params);

    % Initialize arrays to store estimates and weights
    estimates = zeros(params.num_iterations, params.num_vehicles, state_dim);
    weights = ones(params.num_particles, params.num_vehicles) / params.num_particles;

    num_vehicles = params.num_vehicles;
initial_states = struct('d', cell(1, num_vehicles), 'v', cell(1, num_vehicles), 'a', cell(1, num_vehicles), 'D', cell(1, num_vehicles));

for i = 1:num_vehicles
    initial_states(i).d = params.d_stop_line - (i-1) * 50;  % 每辆车间隔50米
    initial_states(i).v = params.v_desired;  % 初始速度设为期望速度
    initial_states(i).a = 0;  % 初始加速度为0
    initial_states(i).D = params.D_undecided;  % 初始决策为未决定
end

    % Generate or load ground truth measurements
    % Replace this with your actual measurement generation or loading process
    % Assuming you have already defined params and initial_states
    [c_tilde, o_tilde, v_avg_tilde, d_tilde] = generate_ground_truth_measurements(params, initial_states);

    % Main particle filter loop
for t = 1:params.num_iterations
    % Predict particles
    particles = PF.predict_particles(particles, params, ST);
    
    % Get measurements for this time step
    measurements = struct();
    measurements.c_tilde = c_tilde(t);
    measurements.o_tilde = o_tilde(t);
    measurements.v_avg_tilde = v_avg_tilde(t);
    measurements.d_tilde = d_tilde(:, t);
    
    % Update weights
    weights = PF.update_weights(particles, measurements, params, M);
    
    % Resample particles
    [particles, weights] = PF.resample_particles(particles, weights);
    
    % Estimate state
    estimates(t, :, :) = PF.state_estimation(particles, weights);
    
    % Optional: Add any additional processing or logging here
end

% Plotting results
figure(3);
figure('Position', [100, 100, 1200, 900]);

state_names = {'Position', 'Velocity', 'Acceleration', 'Decision'};
state_units = {'m', 'm/s', 'm/s^2', ''};

for state = 1:4
    subplot(2, 2, state);
    hold on;
    
    % Plot ground truth (if available)
    % plot(1:params.num_iterations, ground_truth(:, :, state), 'k--', 'LineWidth', 2);
    
    % Plot estimates
    for v = 1:params.num_vehicles
        plot(1:params.num_iterations, squeeze(estimates(:, v, state)), 'LineWidth', 2);
    end
    
    xlabel('Time Step');
    if ~isempty(state_units{state})
        ylabel([state_names{state}, ' (', state_units{state}, ')']);
    else
        ylabel(state_names{state});
    end
    title([state_names{state}, ' Estimates']);
    
    if state == 4  % For decision state
        yticks(1:3);
        yticklabels({'Stop', 'Go', 'Undecided'});
    end
    
    legend_labels = arrayfun(@(x) sprintf('Vehicle %d', x), 1:params.num_vehicles, 'UniformOutput', false);
    % legend(['Ground Truth', legend_labels], 'Location', 'best');
    legend(legend_labels, 'Location', 'best');
    
    grid on;
    hold off;
end

sgtitle('Particle Filter Estimates');

% Ask the user whether to save the graphic
save_choice = input('Do you want to save the figure? (y/n): ', 's');
if strcmpi(save_choice, 'y')
    % Create a folder to save the results
    current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    folder_path = fullfile('C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\results\estimates', char(current_time));
    mkdir(folder_path);
    
    % Save the drawing
    saveas(gcf, fullfile(folder_path, 'particle_filter_estimates.fig'));
    saveas(gcf, fullfile(folder_path, 'particle_filter_estimates.png'));
    
    disp(['The figure has been saved in the folder: ' folder_path]);
else
    disp('Figure was not saved.');
end

close all;