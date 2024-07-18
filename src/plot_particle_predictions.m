function plot_particle_predictions(all_particle_data, all_predictions, num_iterations) % 2个
    % Determine the number of vehicles
    num_vehicles = size(all_predictions, 2);
    
    % Create a figure with subplots for each vehicle
    figure('Position', [100, 100, 1200, 300 * num_vehicles]);
    
    % Generate a colormap for the particles
    particle_colors = jet(num_iterations);
    
    % Generate colors for vehicle prediction lines
    vehicle_colors = hsv(num_vehicles);
    
    for v = 1:num_vehicles
        subplot(num_vehicles, 1, v);
        hold on;
        
        % Plot particles for this vehicle
        for t = 1:num_iterations
            particle_positions = squeeze(all_particle_data(t, :, v, 1));
            scatter(repmat(t, size(particle_positions)), particle_positions, 50, particle_colors(t,:), 'filled', 'MarkerFaceAlpha', 0.7);
        end
        
        % Plot prediction trajectory for this vehicle
        prediction_positions = squeeze(all_predictions(:, v, 1));
        plot(1:num_iterations, prediction_positions, '-', 'Color', vehicle_colors(v,:), 'LineWidth', 2);
        
        % Customize the subplot
        xlabel('Time Step');
        ylabel('Position');
        title(sprintf('Vehicle %d: Positions over Time', v));
        
        % Create legend with correct representations
        particle_handle = scatter(NaN, NaN, 50, 'b', 'filled');  % Off-screen point for legend
        prediction_handle = plot(NaN, NaN, '-', 'Color', vehicle_colors(v,:), 'LineWidth', 2);  % Off-screen line for legend
        legend([particle_handle, prediction_handle], {'Particles', 'Prediction'}, 'Location', 'northwest');
        
        grid on;
        
        % Adjust axis limits
        xlim([0.5, num_iterations + 0.5]);
        ylim([min(all_particle_data(:,:,v,1), [], 'all') - 10, max(all_particle_data(:,:,v,1), [], 'all') + 10]);
        
        hold off;
    end
    
    % Adjust the overall figure layout
    sgtitle('Particle Filter: Positions over Time for All Vehicles');
    tight_layout = true;
    if tight_layout && exist('sgtitle', 'file') == 2  % Check if tight layout and sgtitle are available (MATLAB R2018b or later)
        set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
        pause(0.1);  % Allow time for the figure to update
        try
            tiledlayout('tight');
        catch
            % If tiledlayout is not available, adjust subplot spacing manually
            p = get(gcf, 'Position');
            p(4) = p(4) * 1.1;  % Increase figure height by 10%
            set(gcf, 'Position', p);
            set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
        end
    end
end