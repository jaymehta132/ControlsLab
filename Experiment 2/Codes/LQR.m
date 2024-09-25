Mp = 0.027;         %Mass of assemby
lp = 0.153;         %Length of COM from pivot
Lp = 0.191;         %Total length
r = 0.08260;        %Arm pivot to pendulum pivot
Jm = 3e-5;          %Shaft MOI
Marm = 0.028;       %Mass of arm
g = 9.81;           %Acceleration due to gravity
Jeq = 1.23e-4;      %Equiv. Moment of Inertia about Motor Shaft Pivot Axis
Jp = 1.1e-4;        %Pendulum MOI about pivot axis
Beq = 0;            %Arm viscous damping
Bp = 0;             %Pendulum viscous damping
Rm = 3.3;           %Motor Armature Resistance
Kt = 0.02797;       %Motor torque constant
Km = 0.02797;       %Motor Back-EMF constant

%%

denom = (Jp*Jeq + Mp*lp*lp*Jeq + Jp*Mp*r*r);    %common term in denominator in all terms
A32 = (r*Mp*Mp*lp*lp*g)/denom;
A33 = -(Kt*Km*(Jp+Mp*lp*lp))/(denom*Rm);
A42 = (Mp*lp*g*(Jeq+Mp*r*r))/denom;
A43 = -(Mp*lp*Kt*r*Km)/(denom*Rm);

A = [0 0 1 0;
    0 0 0 1;
    0 A32 A33 0;
    0 A42 A43 0];

%%

B3 = Kt*(Jp + Mp*lp*lp)/(denom*Rm);
B4 = Mp*lp*Kt*r/(denom*Rm);

B = [0;
    0;
    B3;
    B4];

%%

C = eye(4);
D = zeros(4,1);

%%

%LQR Design
Q = diag([1200 2000 20 100]);    %State Penalty Matrix
R = 1;      %Control Penalty Matrix

% K = Optimal Gain Matrix
% S = Solution of Riccati equation
% E = Closed Loop Poles
[K, S, E] = lqr(A,B,Q,R);

% Display K S E
disp('Optimal Gain Matrix');s
disp(K);

disp('Solution of Riccati Equation')
disp(S);

disp('Closed Loop Poles')
disp(E);
%%
sys_c = ss(A, B, C, D);  % Continuous-time system
Ts = 0.001;  % Sampling time (should match the Arduino control rate)
sys_d = c2d(sys_c, Ts, 'zoh');  % Zero-order hold (ZOH) discretization
[A_d, B_d, C_d, D_d] = ssdata(sys_d);  % Discrete-time matrices
