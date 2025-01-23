function [m_c, S_c] = correction(m, S, yv, Svk, C)

    % The function performs a correction step based on the last GPS measurement

    % Arguments:
    % m - current value of the state vector prediction
    % S - current value of the covariance matrix 
    % yv - the measurement on the basis of which the correction is performed
    % Svk - measurement noise matrix
    % C - unit matrix identifying the state variable / state output matrix

    % Outputs:
    % m_c - updated state vector
    % S_c - updated covariance matrix

    % Description:
    % Function performs a correction update after predictions based on 
    % numerical integration (RK4 method). It corrects the latest state
    % vector and covariance matrix in main loop.
    %
    % Dimensions of the input values:
    % m = [pX;pY;pZ;vX;vY;vZ] (6x1)
    % p - position prediction, v - velocity prediction in specified axis
    % S = (6x6)
    % symetrical matrix relative to the diagonal consisting of prediction error
    % yv = [mpX;mpY;mpZ;mvX;mvY;mvZ] (6x1)
    % mp - measured position, v - measured velocity in specified axis
    % Svk = [pErr * eye(3), zeros(3); zeros(3), eye(3) * vErr] (6x6)
    % pErr - measurement noise for position, vErr - measurement noise for velocity
    % C = [eye(3), zeros(3); zeros(3), eye(3)] (6x6)
    % diagonal unit matrix
    %
    % Dimensions of the output values:
    % m_c = [pX;pY;pZ;vX;vY;vZ] (6x1)
    % p - position prediction, v - velocity prediction in specified axis
    % S_c = (6x6)
    % diagonal unit matrix consisting of prediction error
    %
    % Wiktor Pawel, 01.17.2025

    %% Execution
    sigma_k = Svk + C * S * C'; % Covariance matrix of observation prediction error
    L_k = S * C' * inv(sigma_k); % EKF gain
    S_c = S - L_k * sigma_k * L_k';S_c = diag(diag(S_c)); % Update of covariance matrix based on correction
    m_c = m + L_k * (yv - C * m); % Update of state vector based on correction
end