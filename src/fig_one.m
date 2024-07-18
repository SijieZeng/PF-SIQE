figure(1);
    hold on;

    % Generate a colormap for the particles
    colors = jet(num_iterations);
    
    % Plot particles
    for t = 1:num_iterations
        particle_positions = squeeze(all_particle_data(t, :, 1, 1));
        scatter(repmat(t, size(particle_positions)), particle_positions, 50, colors(t,:), 'filled', 'MarkerFaceAlpha', 0.7);
    end
    
    % Plot prediction trajectory
    prediction_positions = squeeze(all_predictions(:, 1, 1));
    plot(1:num_iterations, prediction_positions, 'k-', 'LineWidth', 2);
    
    % Customize the plot
    xlabel('Time Step');
    ylabel('Position');
    title('Particle Filter: Positions over Time');

     % Create legend with correct representations
    particle_handle = scatter(NaN, NaN, 50, 'b', 'filled');  % Off-screen point for legend
    prediction_handle = plot(NaN, NaN, 'k-', 'LineWidth', 2);  % Off-screen line for legend
    legend([particle_handle, prediction_handle], {'Particles', 'Prediction'}, 'Location', 'northwest');
    
    grid on;
    
    % Adjust axis limits
    xlim([0.5, num_iterations + 0.5]);
    ylim([min(all_particle_data(:,:,1,1), [], 'all') - 10, max(all_particle_data(:,:,1,1), [], 'all') + 10]);
    
    hold off;