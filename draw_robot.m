% Draw a car/robot as a rectangle with heading direction
function draw_robot(x, y, theta, length, width, color, style)
    % x, y: position
    % theta: heading angle
    % length: car length (along heading direction)
    % width: car width (perpendicular to heading)
    % color: 'g' for green, 'k' for black, etc.
    % style: 'filled' for solid, 'outline' for just edges
    
    % Car corners in local frame (centered at origin)
    corners_local = [
        length/2,  width/2;   % front-right
        length/2, -width/2;   % front-left
        -length/2, -width/2;  % rear-left
        -length/2,  width/2;  % rear-right
        length/2,  width/2    % close polygon
    ];
    
    % Rotation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Transform to world frame
    corners_world = corners_local * R' + [x, y];
    
    % Draw car body (with HandleVisibility off to avoid legend entries)
    if strcmp(style, 'filled')
        fill(corners_world(:,1), corners_world(:,2), color, 'FaceAlpha', 0.6, 'EdgeColor', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    else
        plot(corners_world(:,1), corners_world(:,2), 'Color', color, 'LineWidth', 2, 'HandleVisibility', 'off');
    end
    
    % Draw heading arrow (front of car)
    arrow_length = length * 0.4;
    arrow_x = x + arrow_length * cos(theta);
    arrow_y = y + arrow_length * sin(theta);
    plot([x arrow_x], [y arrow_y], 'Color', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Arrowhead
    arrow_size = 0.08;
    arrow_angle1 = theta + 2.5;
    arrow_angle2 = theta - 2.5;
    x1 = arrow_x - arrow_size * cos(arrow_angle1);
    y1 = arrow_y - arrow_size * sin(arrow_angle1);
    x2 = arrow_x - arrow_size * cos(arrow_angle2);
    y2 = arrow_y - arrow_size * sin(arrow_angle2);
    plot([x1 arrow_x x2], [y1 arrow_y y2], 'Color', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
end
