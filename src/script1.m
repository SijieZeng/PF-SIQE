% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% Increase graphics timeout
set(0, 'DefaultFigureCreateFcn', @(fig, ~)set(fig, 'CloseRequestFcn', @(src, ~)close(src)));

% State transition Parameters
params = struct();
% Simulation parameters
params.dt = 1;  % time step (s)
params.num_iterations = 100; % Number of time steps
params.num_vehicles = 5; % Number of vehicles 

% generate_traffic_signal_states
params.red_time = 20;   % Red light duration (s)
params.yellow_time = 3.5; % Yellow light duration (s)
params.green_time = 20; % Green light duration (s)
params.d_stop_line = 300; % Stop line position (m)               
params.lane = 1;

% decision making
params.D_h = params.d_stop_line - 150;       % Decision distance in meters
params.T_reaction = 1.0;% Reaction time in seconds
params.p_stop = 0.5;    % Probability of stopping in the dilemma zone
params.b_max = 3.0; 
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
params.s0 = 7.0;        % minimum gap (m), take into account the vehicle length of 5m
params.T = 1.0;         % safe time headway (s)
params.delta = 4;       % acceleration exponent

% Acceleration noise
params.n_a = 0.1;       % Bound for uniform distribution U[-params.n_a, params.n_a]

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



% Set SHOW_TIMING_OUTPUT
SHOW_TIMING_OUTPUT = false; % Set to true if you want to see timing output

% Run simulation
[state, S, total_cpu_times] = simulate(params, SHOW_TIMING_OUTPUT);

% For measuring the time to generate graphics:
cpu_start_graphics = cputime;

% Visualization and store the figures
%folder_path = '/Users/celine/Documents/MATLAB/PF-SIQE_Msc_thesis/results';
folder_path = 'C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\results';



% Position
figure(1);
hold on;
colors = lines(params.num_vehicles);
for i = 1:params.num_vehicles
    plot(1:params.num_iterations, squeeze(state(:, i, 1)), 'Color', colors(i, :));
end

% Plot Dh
plot([1 params.num_iterations], [params.D_h params.D_h], 'k--', 'LineWidth', 2);

% Plot traffic light state on the stop line
for k = 1:params.num_iterations
    if S(k) == "red"
        plot(k, params.d_stop_line, 'r.', 'MarkerSize', 10);
    elseif S(k) == "yellow"
        plot(k, params.d_stop_line, 'y.', 'MarkerSize', 10);
    elseif S(k) == "green"
        plot(k, params.d_stop_line, 'g.', 'MarkerSize', 10);
    end
end

xlabel('Time Step');
ylabel('Position (m)');
title('Vehicle Position and Traffic Signal States Over Time');
legend([arrayfun(@(x) sprintf('Vehicle %d', x), 1:params.num_vehicles, 'UniformOutput', false), 'D_h', 'Traffic Signal']);
ylim([0 params.d_stop_line + 50]);
hold off;

% Interactive axis adjustment
disp('Adjust Position plot axis. Press Enter when done.');
axis manual;
pause;

% Velocity
figure(2);
hold on;
for i = 1:params.num_vehicles
    plot(1:params.num_iterations, squeeze(state(:, i, 2)), 'Color', colors(i, :));
end
xlabel('Time Step');
ylabel('Speed (m/s)');
title('Vehicle Speed Over Time');
legend(arrayfun(@(x) sprintf('Vehicle %d', x), 1:params.num_vehicles, 'UniformOutput', false));
hold off;

% Interactive axis adjustment
disp('Adjust Velocity plot axis. Press Enter when done.');
axis manual;
pause;

% Acceleration
figure(3);
hold on;
for i = 1:params.num_vehicles
    plot(1:params.num_iterations, squeeze(state(:, i, 3)), 'Color', colors(i, :));
end
xlabel('Time Step');
ylabel('Acceleration (m/s^2)');
title('Vehicle Acceleration Over Time');
legend(arrayfun(@(x) sprintf('Vehicle %d', x), 1:params.num_vehicles, 'UniformOutput', false));
hold off;

% Interactive axis adjustment
disp('Adjust Acceleration plot axis. Press Enter when done.');
axis manual;
pause;

% Decisions
figure(4);
hold on;
for i = 1:params.num_vehicles
    decisions = squeeze(state(:, i, 4));
    % Map decision values to numeric values
    decision_map = containers.Map({params.D_stop, params.D_go, params.D_undecided}, {1, 2, 3});
    unique_decisions = unique(decisions);
    disp('Unique decision values:');
    disp(unique_decisions);
    disp('decision_map keys:');
    disp(keys(decision_map));
    
    % create a new mapping to include the 0 value
    extended_decision_map = containers.Map('KeyType', 'double', 'ValueType', 'double');
    extended_decision_map(0) = 0;
    extended_decision_map(params.D_stop) = 1;
    extended_decision_map(params.D_go) = 2;
    extended_decision_map(params.D_undecided) = 3;
    
    mapped_decisions = zeros(size(decisions));
    for idx = 1:numel(decisions)
        if extended_decision_map.isKey(decisions(idx))
        mapped_decisions(idx) = extended_decision_map(decisions(idx));
        else
            warning('Unknown decision value: %d', decisions(idx));
            mapped_decisions(idx) = 0; 
        end
    end
    mapped_decisions = cellfun(@(x) decision_map(x), num2cell(decisions));
    plot(1:params.num_iterations, mapped_decisions, 'Color', colors(i, :));
end
xlabel('Time Step');
ylabel('Decision');
title('Vehicle Decisions Over Time');
yticks([1, 2, 3]);
yticklabels({'Stop', 'Go', 'Undecided'});
ylim([0.5, 3.5]);
legend(arrayfun(@(x) sprintf('Vehicle %d', x), 1:params.num_vehicles, 'UniformOutput', false));
hold off;

% Display a prompt to the user asking if they want to save the figures
save_choice = input('Do you want to save the figures? (y/n): ', 's');

if strcmpi(save_choice, 'y')
    % Create a folder for saving results
    current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    %folder_path = fullfile('/Users/celine/Documents/MATLAB/PF-SIQE_Msc_thesis/results', char(current_time));
    folder_path = fullfile('C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\results', char(current_time));
    mkdir(folder_path);
    
    % Save figures
    figure_titles = {'Vehicle Position', 'Vehicle Speed', 'Vehicle Acceleration', 'Vehicle Decisions'};
    for fig_num = 1:4
        figure(fig_num);
        saveas(gcf, fullfile(folder_path, sprintf('%s_plot.fig', lower(strrep(figure_titles{fig_num}, ' ', '_')))));
    end
    
    disp(['The figures have been saved in the folder: ' folder_path]);
else
    disp('Figures were not saved.');
end

cpu_time_graphics = cputime - cpu_start_graphics;
fprintf('CPU time for generating graphics: %.4f s\n', cpu_time_graphics);

% Close all figures
close all;