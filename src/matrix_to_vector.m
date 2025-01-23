function x_mtv = matrix_to_vector(x, S)

    % Function rewrites a matrix into a vector 

    % Arguments:
    % x - state vector
    % S - covariance matrix

    % Outputs:
    % x_mtv - combined state vector

    % Description:
    % Function rewrites a matrix into a vector. Input matrix is a
    % symetrical matrix relative to the diagonal and it is rewrited with
    % diagonal values as well as components under the diagonal.
    %
    % Dimensions of the input values:
    % x = [pX;pY;pZ;vX;vY;vZ] (6x1)
    % p - position value, v - velocity value in specified axis
    % S = (6x6)
    % symetrical matrix relative to the diagonal consisting of prediction error
    %
    % Dimensions of the output values:
    % x_mtv = (27x1)
    % first 6 rows stands for state vector, remaining 21 rows stand for
    % rewrited covariance matrix
    %
    % Wiktor Pawel, 01.17.2025

    %% Execution
    % Vector initialisation
    x_mtv = zeros(27, 1);

    x_mtv(1:6) = x; % Transcription of estimates
    ind = 7; % First index for matrix in vector
        for i = 1:6
            for j = i:6
                x_mtv(ind) = S(i, j);
                ind = ind + 1;
            end
        end
end
