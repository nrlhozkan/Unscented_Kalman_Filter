% Function to create animation video of robot simulation
% Usage: create_animation(x_store, x_est, P_store, t, landmarks, target_position, num_landmarks)

function create_animation(x_store, x_est, P_store, t, landmarks, target_position, num_landmarks)
    % Load video package
    pkg load video;
    
    % Video recording setup
    video_filename = 'robot_sim.mp4';
    v = VideoWriter(video_filename);
    v.FrameRate = 20;
    open(v);
    
    % Create figure with fixed size
    fig_anim = figure('visible','off', 'Position', [100, 100, 1200, 600]);
    
    % Zoom size for zoomed view (1.2m x 1.2m window)
    zoom_size = 0.6;  % ±0.6m on each side
    
    % Simulation loop
    for k = 2:length(t)
        figure(fig_anim);
        clf;
        
        % Extract current true state
        x_true = x_store(:, k);
        
        % Left subplot: Full trajectory
        subplot(1,2,1);
        hold on; grid on; axis equal;
        xlim([0 25]); ylim([0 25]);
        
        % True and estimated trajectories with explicit DisplayNames
        plot(x_store(1,1:k), x_store(2,1:k), 'g', 'LineWidth', 1.5, 'DisplayName', 'True trajectory');
        plot(x_est(1,1:k), x_est(2,1:k), 'k', 'LineWidth', 1.5, 'DisplayName', 'Estimated trajectory');
        
        % Landmarks and target
        plot(landmarks(:,1), landmarks(:,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Landmarks');
        plot(target_position(1), target_position(2), 'bx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Target');
        
        % Covariance ellipse every 5 steps (no legend entry via HandleVisibility)
        if mod(k,5)==0
            plot_covariance_ellipse(x_est(1:2,k), P_store(1:2,1:2,k), 3, [0.7 0.7 0.7]);
        end
        
        xlabel('X (m)', 'FontSize', 10); 
        ylabel('Y (m)', 'FontSize', 10);
        title('Full Trajectory View', 'FontSize', 11);
        legend('Location','bestoutside', 'FontSize', 9);
        
        % Right subplot: Zoomed robot view (more zoomed: 1.2m x 1.2m)
        subplot(1,2,2);
        hold on; grid on; axis equal;
        
        % Zoom window around true robot (1.2m x 1.2m total = 0.6m on each side)
        xlim([x_true(1)-zoom_size, x_true(1)+zoom_size]);
        ylim([x_true(2)-zoom_size, x_true(2)+zoom_size]);
        
        % Recent trajectory (last 50 points)
        start_idx = max(1, k-50);
        plot(x_store(1,start_idx:k), x_store(2,start_idx:k), 'g-', 'LineWidth', 2.5, 'DisplayName', 'True path');
        plot(x_est(1,start_idx:k), x_est(2,start_idx:k), 'k--', 'LineWidth', 2.5, 'DisplayName', 'Est path');
        
        % Draw true robot as a car (green, filled)
        draw_robot(x_true(1), x_true(2), x_true(3), 0.3, 0.15, 'g', 'filled');
        
        % Draw estimated robot as a car (black, outline)
        draw_robot(x_est(1,k), x_est(2,k), x_est(3,k), 0.3, 0.15, 'k', 'outline');
        
        % Landmarks in view (no legend entry via HandleVisibility)
        for lm = 1:num_landmarks
            if abs(landmarks(lm,1)-x_true(1)) < zoom_size && abs(landmarks(lm,2)-x_true(2)) < zoom_size
                plot(landmarks(lm,1), landmarks(lm,2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
            end
        end
        
        xlabel('X (m)', 'FontSize', 10); 
        ylabel('Y (m)', 'FontSize', 10);
        title('Zoomed Robot View (Green=True, Black=Estimated)', 'FontSize', 11);
        legend('Location','bestoutside', 'FontSize', 9);
        
        % Add text info
        dist_to_target = sqrt((x_true(1)-target_position(1))^2 + (x_true(2)-target_position(2))^2);
        info_text = sprintf('Time: %.2f s\nTrue: [%.2f, %.2f, %.3f]\nEst:  [%.2f, %.2f, %.3f]\nDist: %.2f m', ...
            t(k), x_true(1), x_true(2), x_true(3), ...
            x_est(1,k), x_est(2,k), x_est(3,k), dist_to_target);
        annotation('textbox', [0.01 0.05 0.35 0.25], 'String', info_text, ...
            'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 11, 'FontName', 'monospaced');
        
        % Capture frame
        frame = getframe(fig_anim);
        writeVideo(v, frame);
        %=========================================================%
    end
    
    % Close video
    close(v);
    close(fig_anim);
    
    fprintf('Video saved: %s\n', video_filename);
end
