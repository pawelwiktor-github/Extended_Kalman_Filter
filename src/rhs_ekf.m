function dx = rhs_ekf(t, x)
    
    % The function returns the derivative of the combined state vector

    % Arguments:
    % x - combined, current state vector (27x1) [m(:); S(:)]
    % t - time
    
    % Outputs:
    % dx - derivative of a state vector

    % Description:
    % Function calculates the derivatives of the state vector and matrix covariance according 
    % to the equations in the attached documentation.
    %
    % Dimensions of the input values:
    % x = (27x1)
    % first 6 rows stands for state vector, remaining 21 rows stand for
    % covariance matrix elements
    % t = [t] (1x1)
    % current time stamp
    % Dimensions of the output values:    
    % dx = (27x1) 
    % derivatives by every state vector element as well as every coefficient at diagonal 
    % and components under the diagonal covariance matrix
    %
    % Wiktor Pawel, 01.17.2025

    %% Execution
    % Constants definition
    g = 4.52*1e-5; % Noise intensity
    D = g*g'; % Diffusion matrix

    % Transcription of vector to matrix
    [m, S] = vector_to_matrix(x);

    % Calculation for state vector (m)
    dm = rhs(0, m);

    % Calculation for covariance matrix (S)
    A = get_jacob(0, m);
    dS = A * S + S * A' + D;

    % Transcription of matrix to vector
    dx = matrix_to_vector(dm, dS);
end