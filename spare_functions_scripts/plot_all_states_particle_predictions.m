function plot_all_states_particle_predictions(all_particle_data, all_predictions, num_iterations)

    S = ST.generate_traffic_signal_states(params);
    time_steps_signal = (0:length(S)-1) * params.delta_t;
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
            plot(time_steps_signal(k), params.d_stop_line, [color, '.'], 'MarkerSize', 20);
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
end