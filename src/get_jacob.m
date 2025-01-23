 function A=get_jacob(t,x)
    % The function calculates the Jacobi matrix for the state vector

    % Arguments:
    % t - time
    % x - state vector

    % Outputs:
    % A - Jacobi matrix

    % Description:
    % Function calculates the Jacobi matrix coefficients and performs its
    % filling according to the equations in the attached documentation.
    %
    % Dimensions of the input values:
    % x = [pX;pY;pZ;vX;vY;vZ] (6x1)
    % p - position value, v - velocity value in specified axis
    % t = [t] (1x1)
    % current time stamp
    % Dimensions of the output values:    
    % A = (6x6) 
    % with coefficients filled according to the doc
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
    db_omega=2*omega; % Doubled angular velocity of the Earth's rotation
    omega_square=omega*omega; % Squared angular velocity of the Earth's rotation

    % Filling right half of the matrix  
    A=zeros(6);
    A(1,4)=1;
    A(2,5)=1;
    A(3,6)=1;
    A(4,5)=db_omega;
    A(5,4)=-db_omega;

    % Auxiliary calculations for r
    r2=x(1:3)'*x(1:3);
    r=sqrt(r2);
    r3=r2*r;
    r7=r^7;
    r9=r7*r2;

    % Auxiliary calculations for x
    x12=x(1)^2;
    x22=x(2)^2;
    x32=x(3)^2;
    x13=x(1)*x(3);
    x23=x(2)*x(3);

    % Calculations for auxiliary variables
    % d    
    d=x12+x22;
    % u
    u=6*x32-1.5*d;
    u1=10*x32-7.5*d;

    % q
    q2=[x(1)*u;x(2)*u;x(3)*(3*x32-4.5*d)];
    q3=[x13*u1;x23*u1;4*x32*(x32-3*d)+1.5*d^2];
    q2m(1,1)=u-3*x12;q2m(1,2)=-3*x(1)*x(2);q2m(1,3)=12*x13;
    q2m(2,1)=q2m(1,2);q2m(2,2)=u-3*x22;q2m(2,3)=12*x23;
    q2m(3,1)=-9*x13;q2m(3,2)=-9*x23;q2m(3,3)=9*x32-4.5*d;
    q3m(1,1)=x(3)*u1-15*x12*x(3);q3m(1,2)=-15*x(1)*x23;q3m(1,3)=x(1)*u1+20*x(1)*x32;
    q3m(2,1)=-15*x(1)*x23;q3m(2,2)=x(3)*u1-15*x22*x(3);q3m(2,3)=x(2)*u1+20*x(2)*x32;
    q3m(3,1)=6*x(1)*(d-4*x32);q3m(3,2)=6*x(2)*(d-4*x32);q3m(3,3)=16*x(3)*(x32-1.5*d);

    % Matrix F initialization (gravitational acceleration at point x)
    F=zeros(3,3);

    % Calculation of coefficients for subsequent values of F(i,j)
    c1r=-mu/r3;
    c2r=J2/r7;
    c3r=J3/r9;

    % Calculation of matrix F
    for i=1:3
        for j=1:3
            del_ij=0;if i==j, del_ij=1;end
            F(i,j)=c1r*(del_ij-3*x(i)*x(j)/r2);
            F(i,j)=F(i,j)+(q2m(i,j)-7*q2(i)*x(j)/r2)*c2r;
            F(i,j)=F(i,j)+(q3m(i,j)-9*q3(i)*x(j)/r2)*c3r;
        end
    end
    F(1,1)=F(1,1)+omega_square;
    F(2,2)=F(2,2)+omega_square;

    % Jacobi matrix completion
    A(4:6,1:3)=F;
 end