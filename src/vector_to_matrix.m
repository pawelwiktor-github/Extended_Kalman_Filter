function [x_vtm, S_vtm] = vector_to_matrix(x)

    % Function rewrites a vector into a matrix

    % Arguments:
    % x - combined state vector

    % Outputs:
    % x_vtm - state vector
    % S_vtm - covariance matrix

    % Description:
    % Function rewrites a vector into a matrix. Input vector contains 21
    % rows omitting the state vector, and it stands for diagonal values as
    % well as components under the diagonal for a covariance matrix.
    %
    % Dimensions of the input values:
    % x = (27x1)
    % first 6 rows stands for state vector, remaining 21 rows stand for
    % covariance matrix elements
    %
    % Dimensions of the output values:    
    % x_vtm = [pX;pY;pZ;vX;vY;vZ] (6x1)
    % p - position value, v - velocity value in specified axis
    % S_vtm = (6x6)
    % symetrical matrix relative to the diagonal consisting of prediction error
    %
    % Wiktor Pawel, 01.17.2025

    %% Execution
    x_vtm = x(1:6); % Transcription of estimates
    S_vtm = zeros(6, 6); % Matrix initialisation

    %  Filling the matrix
    ind = 7; % First index from vector
    for i = 1:6
        for j = i:6
            S_vtm(i, j) = x(ind);
            ind = ind + 1;
        end
    end

    % Matrix symmetrization
    S_vtm = S_vtm + S_vtm' - diag(diag(S_vtm));
end