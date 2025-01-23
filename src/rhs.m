 function dx=rhs(t,x)
    % The function calculates the derivatives of the state equations

    % Arguments:
    % t - time
    % x - current state vector

    % Outputs:
    % dx - derivative of a state vector

    % Description:
    % Function calculates the derivatives of the state equations according 
    % to the equations in the attached documentation.
    %
    % Dimensions of the input values:
    % x = [pX;pY;pZ;vX;vY;vZ] (6x1)
    % p - position value, v - velocity value in specified axis
    % t = [t] (1x1)
    % current time stamp
    % Dimensions of the output values:    
    % dx = (6x1) 
    % derivatives by every state vector element
    %
    % Wiktor Pawel, 01.17.2025
 
    %% Execution
    % Constans definition
    mu=398600.4415; % Gravitational parameter [km^3*s^-2]
    Re=6378.1363; % Earth's radius [km]
    Te=86164.09092; % Period of Earth's rotation [s]
    J_2=-0.1082635854*1e-2; % Calculation factor
    J_3=0.2532435346*1e-5; % Calculation factor
    J2=-mu*J_2*Re^2; % Quadrupole flattening of the Earth
    J3=-mu*J_3*Re^3; % Octupole flattening of the Earth
    omega=2*pi/Te; % Angular velocity of the Earth's rotation [rad/s].
    omega_square=omega*omega; % Squared angular velocity of the Earth's rotation

    % State derivative initialization
    dx=zeros(6,1);

    % Auxiliary calculations
    % Gravitational acceleration at a point
    r2=x(1:3)'*x(1:3);
    r=sqrt(r2);
    r3=r2*r;
    r7=r^7;
    r9=r7*r2;
    F1=-mu*x(1:3)/r3;
    d=x(1)^2+x(2)^2;
    x32=x(3)^2;
    u=6*x32-1.5*d;
    F2=J2*x(1:3)/r7;
    F2=F2.*[u;u;3*x32-4.5*d];
    F3=J3*[x(1)*x(3);x(2)*x(3);1]/r9;
    u=10*x32-7.5*d;
    F3=F3.*[u;u;4*x32*(x32-3*d)+1.5*d^2];
    F=F1+F2+F3;

    % Drag forces (in this example not taken into calcualtions)
    Cd=0.0;
    v=x(4:6);
    av=sqrt(v'*v);
    Fd=-Cd*av*v; 

    % State equations of a satellite motion
    dx(1:3)=x(4:6);
    dx(4)=2*omega*x(5)+omega_square*x(1)+F(1)+Fd(1);
    dx(5)=-2*omega*x(4)+omega_square*x(2)+F(2)+Fd(2);
    dx(6)=F(3)+Fd(3);
 end