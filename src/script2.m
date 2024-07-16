% Set up initial states and parameters

% Clear workspace, command window, and close all figures
clear;
%clc;
close all;

% Increase graphics timeout
set(0, 'DefaultFigureCreateFcn', @(fig, ~)set(fig, 'CloseRequestFcn', @(src, ~)close(src)));

% State transition Parameters
params = struct();
% Simulation parameters
params.dt = 1;  % time step (s)
params.num_iterations = 20; % Number of time steps
params.num_vehicles = 3; % Number of vehicles 

% generate_traffic_signal_states
params.red_time = 1;   % Red light duration (s)
params.yellow_time = 3.5; % Yellow light duration (s), cannot tune by now
params.green_time = 10; % Green light duration (s)
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
params.n_a = 0.2;  

% Measurement Parameters
% loop detectors
params.d_loop1 = 200;
params.d_loop2 = 200;
params.d_loop3 = 200;
params.dt_loop1 = 1; % s
params.dt_loop2 = 1; % s
params.dt_loop3 = 1; % s
params.accuracy_loop2 = 0.9;



% Floating sensors
params.sigma_GPS = 5;

ST = state_transition();
M = measurement();
PF = particle_filter_functions();

%initial_states = struct('d', {150, 100, 50}, 'v', {15, 5, 10}, 'a', {0, 0, 0}, 'D', {3, 3, 3});
S = StateTransition.generate_traffic_signal_states(params);
disp('Length of S:');
disp(length(S));
disp('Content of S:');
disp(S);
if ~isstruct(initial_states) || ~isfield(initial_states, 'd') || ~isfield(initial_states, 'v') || ~isfield(initial_states, 'a') || ~isfield(initial_states, 'D')
    error('initial_states must be a struct array with fields d, v, a, and D');
end
% Run simulation
[all_states, all_signals] = simulate1(initial_states, params);
