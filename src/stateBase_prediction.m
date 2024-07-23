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

% Initialize array to store all particle data for all time steps
all_particle_data = zeros(num_iterations, params.num_particles, params.num_vehicles, state_dim);

% Store initial particle data
all_particle_data(1, :, :, :) = particles;

% Initialize array to store predictions for all time steps
all_predictions = zeros(num_iterations, num_vehicles, state_dim);

% Calculate the mean state across all particles
initial_mean_state = squeeze(mean(particles, 1));
if num_vehicles == 1
    initial_mean_state = initial_mean_state(:)';  % Ensure it's a row vector
end

all_predictions(1, :, :) = initial_mean_state;


% Print initial states
disp('Initial States:');
for i = 1:params.num_vehicles
    fprintf('Vehicle %d: d=%.2f, v=%.2f, a=%.2f, D=%d\n', ...
        i, initial_mean_state(i, 1), initial_mean_state(i, 2), ...
        initial_mean_state(i, 3), initial_mean_state(i, 4));
end

% Predict for multiple time steps
for t = 2:num_iterations
    % Predict particles for this time step
    particles = PF.predict_particles(particles, params, ST);

    % Store all particle data for this time step
    all_particle_data(t, :, :, :) = particles;
    
    % Calculate mean state for this time step
    mean_state = squeeze(mean(particles, 1));

    % Ensure mean_state is always a 2D array
    if num_vehicles == 1
        mean_state = mean_state(:)';  % Make it a row vector
    end

    % Calculate mode of decisions
    decisions = squeeze(particles(:, :, 4));
    if num_vehicles == 1
        decision_modes = mode(decisions);
    else
        decision_modes = mode(decisions, 1);
    end

    % Store mean state and mode of decisions for this time step
    all_predictions(t, :, 1:3) = mean_state(:, 1:3);
    all_predictions(t, :, 4) = decision_modes;
    
    % Print predicted states for this time step
    fprintf('\nPredicted States at time step %d:\n', t);
    for i = 1:num_vehicles
         fprintf('Vehicle %d: d=%.2f, v=%.2f, a=%.2f, D=%d\n', ...
            i, mean_state(i, 1), mean_state(i, 2), ...
            mean_state(i, 3), decision_modes(i));
    end
end
%% predictions + particles

    S = ST.generate_traffic_signal_states(params);
    time_steps_signal = (0:length(S)-1) * params.delta_t;


     figure(1);
     figure('Position', [100, 100, 1200, 900]);  % Adjust figure size as needed
    
    state_names = {'Position', 'Velocity', 'Acceleration', 'Decision'};
    state_units = {'m', 'm/s', 'm/s^2', ''};
    
    % Generate a colormap for the particles
    colors = hsv(num_iterations); % jet, parula, hsv, cool
    
    for state = 1:4
        subplot(2, 2, state);
        hold on;
        color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
        for k = 1:length(S)
            color = color_map(S(k));
            line([time_steps_signal(k) time_steps_signal(k)], [params.d_stop_line params.d_stop_line + 5], 'Color', color, 'LineWidth', 2);
            % plot(time_steps_signal(k), params.d_stop_line, [color, '.'], 'MarkerSize', 20);
        end
        % Plot particles
        for t = 1:num_iterations
            particle_values = squeeze(all_particle_data(t, :, 1, state));
            plot(repmat(t, size(particle_values)), particle_values, 'o', 'Color', colors(t,:), 'MarkerSize', 6);
            %scatter(repmat(t, size(particle_values)), particle_values, 30, colors(t,:), 'filled', 'MarkerFaceAlpha', 0.5);
        end
        
        % Plot prediction trajectory
        prediction_values = squeeze(all_predictions(:, 1, state));
        plot(1:num_iterations, prediction_values, 'b--', 'LineWidth', 2);
        
        % Customize the subplot
        xlabel('Time Step');
        if ~isempty(state_units{state})
            ylabel([state_names{state}, ' (', state_units{state}, ')']);
        else
            ylabel(state_names{state});
        end
        title([state_names{state}, ' over Time']);
        
        % Create legend
        if state == 1  % Only add legend to the first subplot to avoid clutter
            particle_handle = plot(NaN, NaN, 'bo', 'MarkerSize', 6);
            % particle_handle = scatter(NaN, NaN, 30, 'b', 'filled');
            prediction_handle = plot(NaN, NaN, 'b--', 'LineWidth', 2);
            legend([particle_handle, prediction_handle], {'Particles', 'Prediction'}, 'Location', 'best');
        end
        
        grid on;
        
        % Adjust axis limits
        xlim([0.5, num_iterations + 0.5]);
        if state ~= 4  % For non-decision states
            y_min = min(all_particle_data(:,:,1,state), [], 'all');
            y_max = max(all_particle_data(:,:,1,state), [], 'all');
            y_range = y_max - y_min;
            ylim([y_min - 0.1*y_range, y_max + 0.1*y_range]);
        else  % For decision state
            ylim([0.5, 3.5]);  % Assuming decisions are 1, 2, or 3
            yticks(1:3);
            yticklabels({'Stop', 'Go', 'Undecided'});  % Adjust these labels as needed
        end
        
        hold off;
    end
    
    % Add an overall title to the figure
    sgtitle('Particle Filter: All States over Time');
    
    % Adjust subplot spacing
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
    try
        set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
    catch
        % If setting fails, ignore and continue
    end
    
    % Add an overall title to the figure
    sgtitle('Particle Filter: State-based Prediction');
    
    % Adjust subplot spacing
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
    try
        set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
    catch
        % If setting fails, ignore and continue
    end

    % Ask the user whether to save the graphic
    save_choice = input('Do you want to save the figure? (y/n): ', 's');
    if strcmpi(save_choice, 'y')
        % Create a folder to save the results
        current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
        folder_path = fullfile('C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\results\state_based_prediction', char(current_time));
        mkdir(folder_path);
        
        % Save the drawing
        saveas(gcf, fullfile(folder_path, 'state_based_prediction.fig'));
        saveas(gcf, fullfile(folder_path, 'state_based_prediction.eps'));
        saveas(gcf, fullfile(folder_path, 'state_based_prediction.png'));
        
        disp(['The figure has been saved in the folder: ' folder_path]);
    else
        disp('Figure was not saved.');
    end
    close all;

%% % Prepare data for plotting; predictions
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
        positions(:, i) = squeeze(all_predictions(:, i, 1));
        velocities(:, i) = squeeze(all_predictions(:, i, 2));
        accelerations(:, i) = squeeze(all_predictions(:, i, 3));
        decisions(:, i) = squeeze(all_predictions(:, i, 4));
    end
end


    % 创建一个新图形
    figure(2);
    figure('Position', [100, 100, 1200, 900]);  % Resize figure to fit subplot
    
    % 1. Vehicle location and traffic signal status
    subplot(2, 2, 1);
    hold on;
    color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
    for k = 1:length(S)
        color = color_map(S(k));
        line([time_steps_signal(k) time_steps_signal(k)], [params.d_stop_line params.d_stop_line + 5], 'Color', color, 'LineWidth', 2);
        % plot(time_steps_signal(k), params.d_stop_line, [color, '.'], 'MarkerSize', 20);
    end
    for i = 1:num_vehicles
        plot(time_steps_vehicle, positions(:, i), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', i));
    end
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Vehicle Positions and Traffic Signals');
    ylim([0, params.d_stop_line + 50]);
    xlim([0, max(time_steps_vehicle(end), time_steps_signal(end))]);
    legend_elements = [plot(NaN,NaN,'r.','MarkerSize',20), plot(NaN,NaN,'y.','MarkerSize',20), plot(NaN,NaN,'g.','MarkerSize',20)];
    legend([legend_elements, plot(NaN,NaN,'-','Color','b')], {'Red', 'Yellow', 'Green', 'Vehicle'}, 'Location', 'best');
    grid on;
    
    % 2. Vehicle speed
    subplot(2, 2, 2);
    hold on;
    color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
    for k = 1:length(S)
        color = color_map(S(k));
        line([time_steps_signal(k) time_steps_signal(k)], [-0.7 -0.3], 'Color', color, 'LineWidth', 2);
        % plot(time_steps_signal(k), -0.5, [color, '.'], 'MarkerSize', 20);
    end
    for v = 1:num_vehicles
        plot(time_steps_vehicle, velocities(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
    end
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Vehicle Velocities');
    legend('Location', 'best');
    grid on;
    
    % 3. Vehicle acceleration
    subplot(2, 2, 3);
    hold on;
    color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
    for k = 1:length(S)
        color = color_map(S(k));
        line([time_steps_signal(k) time_steps_signal(k)], [-1.65 -1.6], 'Color', color, 'LineWidth', 2);
        % plot(time_steps_signal(k), -1.6, [color, '.'], 'MarkerSize', 20);
    end
    for v = 1:num_vehicles
        plot(time_steps_vehicle, accelerations(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
    end
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    title('Vehicle Accelerations');
    legend('Location', 'best');
    grid on;
    
    % 4. Vehicle Decision
    subplot(2, 2, 4);
    hold on;
    color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
    for k = 1:length(S)
        color = color_map(S(k));
        line([time_steps_signal(k) time_steps_signal(k)], [0.85 0.9], 'Color', color, 'LineWidth', 2);
        % plot(time_steps_signal(k), 0.9, [color, '.'], 'MarkerSize', 20);
    end
    for v = 1:num_vehicles
        plot(time_steps_vehicle , decisions(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
    end
    xlabel('Time (s)');
    ylabel('Decision');
    title('Vehicle Decisions');
    yticks([1, 2, 3]);
    yticklabels({'Stop', 'Go', 'Undecided'});
    legend('Location', 'best');
    grid on;
    
    % Adjust the spacing between subplots
    spacing = 0.05;
    subplot_pos = get(subplot(2,2,1), 'Position');
    set(subplot(2,2,1), 'Position', [subplot_pos(1), subplot_pos(2), subplot_pos(3)-spacing, subplot_pos(4)-spacing]);
    subplot_pos = get(subplot(2,2,2), 'Position');
    set(subplot(2,2,2), 'Position', [subplot_pos(1)+spacing, subplot_pos(2), subplot_pos(3)-spacing, subplot_pos(4)-spacing]);
    subplot_pos = get(subplot(2,2,3), 'Position');
    set(subplot(2,2,3), 'Position', [subplot_pos(1), subplot_pos(2)+spacing, subplot_pos(3)-spacing, subplot_pos(4)-spacing]);
    subplot_pos = get(subplot(2,2,4), 'Position');
    set(subplot(2,2,4), 'Position', [subplot_pos(1)+spacing, subplot_pos(2)+spacing, subplot_pos(3)-spacing, subplot_pos(4)-spacing]);
    
    % Add a general title
    sgtitle('State based predictions ', 'FontSize', 16);
    
    % Ask the user whether to save the graphic
    save_choice = input('Do you want to save the figure? (y/n): ', 's');
    if strcmpi(save_choice, 'y')
        % Create a folder to save the results
        current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
        folder_path = fullfile('C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\results\state_based_prediction\predictions-3vehicles-100particles', char(current_time));
        mkdir(folder_path);
        
        % Save the drawing
        saveas(gcf, fullfile(folder_path, 'state_based_prediction.fig'));
        saveas(gcf, fullfile(folder_path, 'state_based_prediction.eps'));
        saveas(gcf, fullfile(folder_path, 'state_based_prediction.png'));
        
        disp(['The figure has been saved in the folder: ' folder_path]);
    else
        disp('Figure was not saved.');
    end
    
    close all;
   