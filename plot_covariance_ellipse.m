function h = plot_covariance_ellipse(mu, Sigma, n_sigma, color)
    % Plot covariance ellipse
    % mu: 2x1 mean vector [x; y]
    % Sigma: 2x2 covariance matrix
    % n_sigma: number of standard deviations (e.g., 3 for 3-sigma)
    % color: ellipse color (e.g., 'b', 'r', [0.5 0.5 0.5])
    % h: handle to the filled patch (optional output)
    
    % Eigenvalue decomposition
    [V, D] = eig(Sigma);
    
    % Get the angle of the major axis
    angle = atan2(V(2,1), V(1,1));
    
    % Get the lengths of the semi-axes (scaled by n_sigma)
    a = n_sigma * sqrt(D(1,1)); 
    b = n_sigma * sqrt(D(2,2)); 
    
    % Generate ellipse points
    theta = linspace(0, 2*pi, 100);
    ellipse = [a*cos(theta); b*sin(theta)];
    
    % Rotation matrix
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    
    % Rotate and translate
    ellipse_rot = R * ellipse;
    x_ellipse = mu(1) + ellipse_rot(1,:);
    y_ellipse = mu(2) + ellipse_rot(2,:);
    
    % Plot filled ellipse (HandleVisibility off to avoid legend entries)
    h = fill(x_ellipse, y_ellipse, color, 'FaceAlpha', 0.7, 'EdgeColor', color, 'LineWidth', 1, 'HandleVisibility', 'off');
end
