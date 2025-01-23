function [t,x]=rk4(x0,tf,SPTu)

    % The function performs RK4 integration

    % Arguments:
    % x0 - current state vector
    % tf - time window over which the integration will be calculated
    % SPTu - number of steps per time unit

    % Outputs:
    % x - integrated state vector
    % t - time of integration

    % Description:
    % Function calculates the integral of state vector based on
    % Rungego-Kutty method (4th row).
    %
    % Dimensions of the input values:
    % x0 = (27x1)
    % first 6 rows stands for state vector, remaining 21 rows stand for
    % covariance matrix elements
    % t = [t] (1x1)
    % current time stamp
    % Dimensions of the output values:    
    % t = [t] (1x1)
    % time of integration
    % x = (27x1) 
    % integral by every state vector element as well as every coefficient at diagonal 
    % and components under the diagonal covariance matrix
    %
    % Wiktor Pawel, 01.17.2025

    %% Execution
    % Basic calculation
    n = length(x0);
    nt = 1 + floor(tf * SPTu);
    h = tf / nt;
    h_2 = h / 2; h_6 = h / 6; h_26 = 2 * h_6;

    % Parameters initialization
    x0=x0(:);
    x=zeros(nt+1,n);
    t=zeros(nt+1,1);
    x(1,:)=x0';
    tmp=zeros(n,1);
    xtmp=x0;tt=0;
    dx1=zeros(n,1);
    dx2=zeros(n,1);
    dx3=zeros(n,1);
    dx4=zeros(n,1);

    for i=1:nt
        dx1=rhs_ekf(tt,xtmp);tmp=xtmp+h_2*dx1;tt=tt+h_2;
        dx2=rhs_ekf(tt,tmp);tmp=xtmp+h_2*dx2;
        dx3=rhs_ekf(tt,tmp);tmp=xtmp+h*dx3;tt=tt+h_2;
        dx4=rhs_ekf(tt,tmp);
        xtmp=xtmp+h_6*(dx1+dx4)+h_26*(dx2+dx3);
        x(i+1,:)=xtmp';t(i+1)=tt;
    end
end