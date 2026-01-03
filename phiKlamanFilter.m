function s = phiKlamanFilter(s, u)

    s.x = s.A*s.x + s.B*u;
    s.P = s.A*s.P*s.A' + s.Q;

    % compute kalman gain
    K = s.P*s.H'*inv(s.H*s.P*s.H' + s.R);
    % update state estimate and error covariance
    s.x = s.x + K*(s.z - s.H*s.x);
    s.P = s.P - K*s.H*s.P;

end