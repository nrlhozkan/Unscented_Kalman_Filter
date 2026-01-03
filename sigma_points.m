function [chi] = sigma_points(x, P, lambda, n)
    % Calculate square root of (n+lambda)*P
    S = chol((n + lambda) * P, 'lower');

    % Initialize sigma points matrix
    chi = zeros(n, 2*n + 1);
    chi(:,1) = x; % First sigma point is the mean
    for i = 1:n
        chi(:,i+1) = x + S(:,i);       % Sigma points in positive direction
        chi(:,i+1+n) = x - S(:,i);     % Sigma points in negative direction
    end
end