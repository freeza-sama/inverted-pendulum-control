clc ; clear; close all


M_w = 2.9;   % Mass of one wheel
M_r = 12.09;   % Mass of robot body
m_b = 2.78/1000;  % Mass of the ball
R = 0.282;    % Radius of the wheel
h = 0.26681;     % Height of robot CoM from ball platform
r = 40.02/1000;   % Radius of the ball
g = 9.81;    % Gravity
d = 0.12481; 

% Safety limits
x_limit = d/4;                 % ball can move only within +/- d/2
theta_limit = deg2rad(15);     % 15 degrees in radians

% --- 2. Moments of Inertia ---
I_w = 0.5 * M_w * R^2;        % Inertia of one wheel (Note: Not used in M1, M2, M3)
I_r_com = (1/12) * M_r * (h^2 + d^2);       % body about its own COM
I_r = I_r_com + M_r * d^2;              % Inertia of robot body (parallel axis theorem)
I_b = 0.4 * m_b * r^2;      % Inertia of a solid sphere ball

% % --- 3. Derived Parameters ---
l = h + r; % Height of ball CoM from wheel center
% 
% 
M1 = [(I_b/(r^2)) + m_b, m_b*l;
      m_b*l,            I_r + m_b*(l^2)];

% M2 = [[1,  1+(g*m_b)],
%       [1-(m_b*g), 1 - (M_r*d*g)-(m_b*g*l)]]
M2 = [0,             (g*m_b);
      (m_b*g), (M_r*d*g) + (m_b*g*l)];

% M3 = [[m_b*l],
%       [-m_b]]
M3 = -[m_b*l;
      I_r+ m_b*l^2];

% --- 5. Calculate A and B ---
% This is the numerically preferred method for A = inv(M1) * M2
A1 = M1 \ M2;

% This is the numerically preferred method for B = inv(M1) * M3
B1 = M1 \ M3;

A = [0 1 0 0;...
    A1(1,1) 0 A1(1,2) 0;...
    0 0 0 1;...
    A1(2,1) 0 A1(2,2) 0];
B = -[0 ; B1(1); 0 ; B1(2)];
C = eye(4);
D = zeros(4,1);

[num,den] = ss2tf(A,B,C,D)

function info = impulseinfo(t, y)
    % simplified impulse-response info
    % fields: Peak, SettlingTime, Overshoot

    y = y(:);
    t = t(:);

    % Peak
    [Peak, ~] = max(abs(y));

    % Final value
    final_val = y(end);

    % Overshoot (only meaningful if final != 0)
    if abs(final_val) > 1e-6
        Overshoot = (Peak - abs(final_val)) / abs(final_val);
    else
        Overshoot = NaN;   % consistent with control theory practice
    end

    % Settling time (2% band)
    err = abs(y - final_val);
    tol = 0.02 * abs(final_val);
    idx_set = find(err <= tol, 1, 'first');
    if ~isempty(idx_set)
        SettlingTime = t(idx_set);
    else
        SettlingTime = NaN;
    end

    info = struct('Peak', Peak, ...
                  'SettlingTime', SettlingTime, ...
                  'Overshoot', Overshoot);
end


s = tf('s');
G_theta = tf(num(3,:), den);
Kp_t = 2000; % Proportional gain
Ki_t = 10; % Integral gain
Kd_t = 25; % Derivative gain
C_theta = pid(Kp_t, Ki_t, Kd_t);
T_theta = feedback(C_theta*G_theta, 1);
pole(T_theta)
impulse(T_theta)

sys = ss(A,B,C,D);     % 1 input (u), 4 outputs: [x; dx; θ; dθ]
C_theta = pid(Kp_t, Ki_t, Kd_t);   % tuned so θ response is fast & well damped
% inner_cl: input = θ_ref, outputs = [x; dx; θ; dθ]
inner_cl = feedback(sys, C_theta, 1,3);

poles_inner = pole(inner_cl);
fprintf('Inner loop poles:\n');
disp(poles_inner);

% Check that all poles are in LHP (real part < 0)
if all(real(poles_inner) < 0)
    fprintf('Inner loop is STABLE ✓\n');
else
    fprintf('Inner loop is UNSTABLE ✗\n');
end


figure;
impulse(inner_cl(3,1), 2);  % θ (output 3) due to θ_ref (input 1)
title('Inner Loop: θ Response to θ_ref Step');
grid on;

% Get transfer function from θ_ref to x
Gx_outer = tf(inner_cl(1,1));  % x (output 1) / θ_ref (input 1)
Gx_outer = minreal(Gx_outer);  % clean up

fprintf('Outer loop plant (θ_ref → x):\n');
disp(Gx_outer);

% Check its poles
poles_outer = pole(Gx_outer);
fprintf('Outer plant poles:\n');
disp(poles_outer);

% Quick sanity check
figure; impulse(Gx_outer, 2);
title('Outer Plant: x Response to θ_ref');
grid on;

% Design outer controller
figure; rlocus(Gx_outer); grid on;
title('Root Locus for Outer Loop (x control)');


Kp_x = 5.0;
Ki_x = .01000;
Kd_x = 41;


C_x = pid(Kp_x, Ki_x, Kd_x);
T_x = feedback(-C_x*Gx_outer, 1);   % minus sign here

impulse(T_x); grid on;
pole(T_x)

% Combine both loops
C_x = -pid(Kp_x, Ki_x, Kd_x);  % your tuned outer controller
G_total = series(C_x, inner_cl);  % x_ref → θ_ref → [x;...]
T_total = feedback(G_total(1,1), 1);  % close outer feedback

% Simulate step response
figure; impulse(T_total, 700);
title('Complete Ballbot: x Response to x_ref');
grid on; hold on;
xlabel('Time (s)'); ylabel('Ball Position x (m)');

% Check performance
stepinfo(T_total)

[num_x, den_x] = tfdata(Gx_outer, 'v');
max_pos_error = 0.5;
max_vel_error = 1.0;
max_tilt_error = 5*pi/180;
max_tilt_rate = 1.0;
max_ball_error = 0.05;
max_ball_vel = 0.5;
max_torque = 2.0;


Q_x      = 1/max_pos_error^2;       % Low priority: It's okay if the robot drives far away
Q_theta  = 1/max_tilt_error^2;    % Medium-High: Don't fall over, but tilting is allowed
Q_s      = 1/max_ball_error^2;    % HIGH PRIORITY: Keep the ball in the center!

% Velocity penalties (Damping)
Q_vx     = 1/max_vel_error^2; 
Q_vtheta = 1/max_tilt_rate^2;
Q_vs     = 1/max_ball_vel ^2;

Q = diag([Q_x,Q_vx,Q_theta,Q_vtheta]);
R_lqr= 100/max_torque^2;

% =======================================================
% PASTE THIS AT THE VERY BOTTOM OF YOUR EXISTING CODE
% =======================================================

% --- 1. Design the LQR Controller (The missing part) ---
% We need to compare your PID against this.
 % Penalties on [x, dx, theta, dtheta]
                 % Penalty on motor effort
K_lqr = lqr(A, B, Q, R_lqr);

% Create the LQR closed-loop system
% We add Nbar to make sure it tracks the reference correctly
sys_cl_lqr = ss(A - B*K_lqr, B, C, D);
Nbar = 1 / dcgain(sys_cl_lqr(1,1)); % Scale factor
sys_cl_lqr_scaled = ss(A - B*K_lqr, B*Nbar, C, D);

% --- 2. Compare Impulse Responses (The required plot) ---
figure;
hold on;
% Plot your PID result (T_total)
impulse(T_total, 10); 
% Plot the LQR result
impulse(sys_cl_lqr_scaled(1,1), 10);

legend('Your PID', 'LQR (Full State Feedback)');
title('Comparison: PID vs LQR Response');
grid on;

% --- 3. Numerical Comparison (Command Window Output) ---
info_PID = stepinfo(T_total);            % Get metrics for your PID
info_LQR = stepinfo(sys_cl_lqr_scaled(1,1)); % Get metrics for LQR

fprintf('\n=========================================\n');
fprintf('      ASSIGNMENT COMPARISON TABLE        \n');
fprintf('=========================================\n');
fprintf('%-15s | %-12s | %-12s\n', 'Metric', 'Your PID', 'LQR');
fprintf('-----------------------------------------\n');
% Note: Impulse response metrics are slightly different than step, 
% but this provides the numeric data requested.
fprintf('%-15s | %-12.4f | %-12.4f\n', 'SettlingTime', info_PID.SettlingTime, info_LQR.SettlingTime);
fprintf('%-15s | %-12.4f | %-12.4f\n', 'Peak Value', info_PID.Peak, info_LQR.Peak);
fprintf('-----------------------------------------\n');
fprintf('Comment: LQR usually settles faster with less oscillation.\n');