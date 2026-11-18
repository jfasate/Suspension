clc
clear
close all

load('road_input.mat') % Load pre-saved road profile disturbance data

% Load Fuzzy Inference System (FIS) controllers from local files

susp_fis_cont=readfis('FUZZY_CONTROL.fis');      % FIS for active actuator force control

% Symbols for Lagrangian Symbolic Derivation
syms zs theta phi zu1 zu2 zu3 zu4 real              
% zs: Sprung mass heave (vertical displacement of the car body)
% theta: Sprung mass pitch angle (front-to-back rotation)
% phi: Sprung mass roll angle (side-to-side rotation)
% zu1, zu2, zu3, zu4: Unsprung mass vertical displacements (Front-Left, Front-Right, Rear-Left, Rear-Right)

syms dzs dtheta dphi dzu1 dzu2 dzu3 dzu4 real       
% dzs: Sprung mass vertical velocity (heave velocity)
% dtheta: Sprung mass pitch velocity (angular rate)
% dphi: Sprung mass roll velocity (angular rate)
% dzu1, dzu2, dzu3, dzu4: Unsprung mass vertical velocities (FL, FR, RL, RR wheels)

syms ms mu1 mu2 mu3 mu4 Is Ir real                  
% ms: Total mass of the sprung body (chassis/car body)
% mu1, mu2, mu3, mu4: Unsprung masses (individual wheel, tire, and axle assembly masses)
% Is: Pitch moment of inertia of the sprung mass
% Ir: Roll moment of inertia of the sprung mass

syms l1 l2 w1 w2 c1 c2 c3 c4 real                   
% l1: Distance from the vehicle Center of Gravity (CG) to the front axle
% l2: Distance from the vehicle Center of Gravity (CG) to the rear axle
% w1: Front half-track width (distance from CG to front left/right wheels)
% w2: Rear half-track width (distance from CG to rear left/right wheels)
% c1, c2, c3, c4: Passive damper/shock absorber damping coefficients (FL, FR, RL, RR)

syms ks1 ks2 ks3 ks4 kt1 kt2 kt3 kt4 real           
% ks1, ks2, ks3, ks4: Suspension helical spring stiffness constants (FL, FR, RL, RR)
% kt1, kt2, kt3, kt4: Vertical tire/tyre stiffness constants (FL, FR, RL, RR)

syms  r1 r2 r3 r4 F1 F2 F3 F4 real
% r1, r2, r3, r4: Road profile height profile disturbances at each wheel hub (FL, FR, RL, RR)
% F1, F2, F3, F4: Active actuator control forces acting between sprung and unsprung masses (FL, FR, RL, RR)

% Generalized coordinates & derivatives
q   = [zs;   theta;   phi; zu1;   zu2;   zu3;   zu4]; % Generalised displacement vector
qd  = [dzs;  dtheta;  dphi; dzu1;  dzu2;   dzu3;  dzu4]; % Generalised velocity vector
qdd = sym('qdd', [7 1]); % Generalized acceleration vector template

% Spring elongation (Relative suspension deflections)
Vs1 = zs + l1*theta + w1*phi - zu1;   % front-left suspension deflection
Vs2 = zs + l1*theta - w1*phi - zu2;   % front-right suspension deflection
Vs3 = zs - l2*theta + w2*phi - zu3;   % rear-left suspension deflection
Vs4 = zs - l2*theta - w2*phi - zu4;   % rear-right suspension deflection

% Damper elongation rates (Relative suspension deflection velocities)
Vsd1 = dzs + l1*dtheta + w1*dphi - dzu1; % front-left damper velocity
Vsd2 = dzs + l1*dtheta - w1*dphi - dzu2; % front-right damper velocity
Vsd3 = dzs - l2*dtheta + w2*dphi - dzu3; % rear-left damper velocity
Vsd4 = dzs - l2*dtheta - w2*dphi - dzu4; % rear-right damper velocity

% System Energies
% T: Total Kinetic Energy (Translational + Rotational energies of sprung and unsprung masses)
T = 1/2*ms*dzs^2 + 1/2*Is*dtheta^2 + 1/2*Ir*dphi^2 + 1/2*mu1*dzu1^2 + 1/2*mu2*dzu2^2 + 1/2*mu3*dzu3^2 + 1/2*mu4*dzu4^2;

% V: Total Potential Energy (Elastic strain energy stored in suspension springs and tires)
V = 1/2*ks1*Vs1^2 + 1/2*ks2*Vs2^2 + 1/2*ks3*Vs3^2 + 1/2*ks4*Vs4^2 + 1/2*kt1*(zu1 - r1)^2 + 1/2*kt2*(zu2 - r2)^2 + 1/2*kt3*(zu3 - r3)^2 + 1/2*kt4*(zu4 - r4)^2;

% F: Rayleigh non-conservative dissipation function (Energy loss rate due to dampers)
F = 1/2*c1*Vsd1^2 + 1/2*c2*Vsd2^2 + 1/2*c3*Vsd3^2 + 1/2*c4*Vsd4^2;

% L: Lagrangian operator (Difference between Kinetic and Potential energy)
L = T - V;

% Q: Generalized forces mapping vector corresponding to active force inputs (F1-F4)
Q = [ F1 + F2 + F3 + F4;
      l1*F1 + l1*F2 - l2*F3 - l2*F4;
      w1*F1 - w1*F2 + w2*F3 - w2*F4;
      -F1;
      -F2;
      -F3;
      -F4 ];

% Lagrange equations solver loop
eqns = sym(zeros(4,1));
for i = 1:7
   
    dL_dqi  = diff(L, q(i));   % Partial derivative of L with respect to generalised coordinates
    dL_dqdi = diff(L, qd(i));  % Partial derivative of L with respect to generalised velocities
    ddt = 0;
    for j = 1:7
        % Chain rule implementation to obtain total time derivative d/dt(∂L/∂q̇_i)
        ddt = ddt + diff(dL_dqdi, q(j))*qd(j) + diff(dL_dqdi, qd(j))*qdd(j);
    end
    % Rayleigh dis. force
    dF_dqdi = diff(F, qd(i));
    % Lagrange Equation Equation
    eqns(i) = simplify(ddt - dL_dqi + dF_dqdi - Q(i));
end

% Solve the system of algebraic equations for accelerations (qdd)
sol = solve(eqns == 0, qdd);

% Display symbolic results in Command Window
disp('ddzs =');     disp(simplify(sol.qdd1));  % Sprung mass heave acceleration
disp('ddtheta =');  disp(simplify(sol.qdd2));  % Sprung mass pitch angular acceleration
disp('ddphi =');    disp(simplify(sol.qdd3));  % Sprung mass roll angular acceleration
disp('ddzu1 =');    disp(simplify(sol.qdd4));  % Front-Left wheel acceleration
disp('ddzu2 =');    disp(simplify(sol.qdd5));  % Front-Right wheel acceleration
disp('ddzu3 =');    disp(simplify(sol.qdd6));  % Rear-Left wheel acceleration
disp('ddzu4 =');    disp(simplify(sol.qdd7));  % Rear-Right wheel acceleration

% System Parameters
ms = 1000;      % Total Sprung Mass (kg)
Is = 1200;      % Pitch moment of inertia of body (kg*m^2)
Ir = 1200;      % Roll moment of inertia of body (kg*m^2)
mu1 = 70;       % Unsprung Mass - Front Left assembly (kg)
mu2 = 70;       % Unsprung Mass - Front Right assembly (kg)
mu3 = 70;       % Unsprung Mass - Rear Left assembly (kg)
mu4 = 70;       % Unsprung Mass - Rear Right assembly (kg)
l1 = 1.2;       % Distance from CG to front axle line (m)
l2 = 1.5;       % Distance from CG to rear axle line (m)
w1 = 0.7;       % Distance from CG to front longitudinal wheel track line (m)
w2 = 0.7;       % Distance from CG to rear longitudinal wheel track line (m)
ks1 = 30000;    % Suspension spring stiffness - Front Left (N/m)
ks2 = 30000;    % Suspension spring stiffness - Front Right (N/m)
ks3 = 30000;    % Suspension spring stiffness - Rear Left (N/m)
ks4 = 30000;    % Suspension spring stiffness - Rear Right (N/m)
kt1 = 200000;   % Tire radial/vertical stiffness - Front Left (N/m)
kt2 = 200000;   % Tire radial/vertical stiffness - Front Right (N/m)
kt3 = 200000;   % Tire radial/vertical stiffness - Rear Left (N/m)
kt4 = 200000;   % Tire radial/vertical stiffness - Rear Right (N/m)
c1=1502;        % Shock absorber damping coefficient - Front Left (Ns/m)
c2=1502;        % Shock absorber damping coefficient - Front Right (Ns/m)
c3=1502;        % Shock absorber damping coefficient - Rear Left (Ns/m)
c4=1502;        % Shock absorber damping coefficient - Rear Right (Ns/m)

dT=0.001;       % Simulation fixed time-step discrete size (s)

Fuzzy_Gain=2300;% Output scaling mapping multiplier factor for Fuzzy Logic Controller
Kp=23750;       % Proportional Gain for parallel active suspension PID block
Ki = 250000;    % Integral Gain for parallel active suspension PID block
Kd = 2873;      % Derivative Gain for parallel active suspension PID block
fc=50;          % Cutoff frequency for LPF
%%
%extract the outputs in the Command Window
%and then copy and paste them into a MATLAB Function block in Simulink.

%% Random Filtered Waveform for Gravel Road Effect

% fs = 1000;
% t = 0:1/fs:10;
% raw_noise = randn(size(t));
% [b, a] = butter(2, 15/(fs/2));
% r1 = 0.03 * filter(b, a, raw_noise);
% r2 = 0.03 * filter(b, a, randn(size(t)));
% 
% % Timeseries object
% ts_r1 = timeseries(r1', t'); 
% ts_r2 = timeseries(r2', t');
% 
% save('road_input.mat', 'ts_r1', 'ts_r2', '-v7.3');
% 
% 
% Z= [max(out.z1) min(out.z1);
%     max(out.z2) min(out.z2);
%     max(out.z3) min(out.z3);
%     max(out.z4) min(out.z4)]
% D= [max(out.d1) min(out.d1);
%     max(out.d2) min(out.d2);
%     max(out.d3) min(out.d3);
%     max(out.d4) min(out.d4)]
%
