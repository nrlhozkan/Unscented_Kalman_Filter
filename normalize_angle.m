% def normalize_angle(x):
%     x = x % (2 * np.pi)  # force in range [0, 2 pi)
%     if x > np.pi:  # move to [-pi, pi)
%         x -= 2 * np.pi
% return x


function angle = normalize_angle(angle)
    angle = mod(angle, 2 * pi); % Force in range [0, 2*pi)
    if angle > pi
        angle = angle - 2 * pi; % Move to [-pi, pi)
    end
end