clc; clear;close all;
Mr=12.09; d=0.12481; h=0.26681; R=0.282; Mw=2.9; Mb=2.78/1000; r=40.02/1000; g = 9.81;

syms x(t) theta(t) s(t) t T_input(t)

%wheel 
x_wheel=x;
Iw = 0.5*Mw*R^2;
vel_wheel = diff(x_wheel,t);
omega_wheel = vel_wheel/R;
KEwheel = 0.5*Mw*(vel_wheel)^2+0.5*Iw*(omega_wheel)^2;

%robot
x_robot = x+d*sin(theta);y_robot = d*cos(theta);
x_vel_bot=diff(x_robot,t); y_vel_bot = diff(y_robot,t);
Ir=Mr*(d^2)+(Mr/12)*(h^2+d^2);
omega_bot = diff(theta,t);
PEbot = Mr*g*y_robot;
KEbot = 0.5*Mr*(x_vel_bot^2+y_vel_bot^2) + 0.5*Ir*(omega_bot^2);

%ball
x_ball = x+(h+r)*sin(theta)+s*cos(theta); y_ball = (h+r)*cos(theta)-s*sin(theta);
x_vel_ball = diff(x_ball,t); y_vel_ball = diff(y_ball,t);
Ib = 0.5*Mb*r^2;
omega_ball = diff(s,t)/r;
PEball = Mb*g*y_ball;
KEball = 0.5*Mb*(x_vel_ball^2+y_vel_ball^2)+0.5*Ib*(omega_ball^2);

%lagrange
L=(KEball+KEbot+KEwheel) - (PEball+PEbot);

q = [x(t),theta(t),s(t)];
dq = diff(q,t);
ddq = diff(q,t,2);

syms x_stat theta_stat s_stat T_stat
syms vx vtheta vs
syms ax atheta as
dq_dummy = [vx vtheta vs];
ddq_dummy = [ax atheta as];
Q = [T_input(t)/R -T_input(t) 0];

eqn =sym(zeros(3,1));
for i=1:3
    L_dq_dummy = subs(L,dq,dq_dummy); %creates a lagnrage with variables and no diff
    dL_dq_dummy = diff(L_dq_dummy,dq_dummy(i)); % this is the first term b4 time derivative dL/dxdot
    dL_dq_real = subs(dL_dq_dummy,dq_dummy,dq); % subs back the dL/dxdot back to diff from 
    ddL_dt = diff(dL_dq_real,t); % fidns the first term d(dL/dxdot)/dt
    dL_q = diff(L,q(i));
    
    eqn(i) = ddL_dt - dL_q==Q(i); % gets the answer in terms of diff of all terms

end 

% to solve for the accelerations

Soln = subs(eqn,ddq,ddq_dummy);
acc_dummy = solve(Soln,ddq_dummy);

f_ax = acc_dummy.ax;
f_atheta = acc_dummy.atheta;
f_as = acc_dummy.as;

%Linearisation 
calc_vars = [x(t), diff(x(t),t), theta(t), diff(theta(t),t), s(t), diff(s(t),t), T_input(t)];
state_vars = [x_stat, vx, theta_stat,vtheta,s_stat,vs,T_stat];

f_ax_lin = subs(f_ax,calc_vars,state_vars);
f_atheta_lin = subs(f_atheta,calc_vars,state_vars);
f_as_lin = subs(f_as,calc_vars,state_vars);

df = [vx;f_ax_lin;vtheta;f_atheta_lin;vs;f_as_lin];
f = [x_stat; vx; theta_stat;vtheta;s_stat;vs];

point = [0 0 0 0 0 0 0];

A_dummy = jacobian(df,f);
A_real = subs(A_dummy,state_vars,point);
A_real = double(A_real);

B_dummy = jacobian(df,T_stat);
B_real = subs(B_dummy,state_vars,point);
B_real = double(B_real);

 C = [1 0 0 0 0 0;0 0 1 0 0 0];
%C = [0 0 1 0 0 0];
D = 0;

% Prioritize keeping the ball in the center (s=0)
% Allow the robot to tilt (theta) and move (x) to achieve this.\

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

Q = diag([Q_x,Q_vx,Q_theta,Q_vtheta,Q_s,Q_vs]);
R_lqr= 100/max_torque^2; % Allow reasonable motor power

K = lqr(A_real,B_real,Q,R_lqr);

x_ref = [0; 0; 0; 0; 0; 0];

%ode45 

accel_function = matlabFunction(f_ax_lin, f_atheta_lin, f_as_lin, 'Vars', {state_vars});

% 2. Run ODE45
t_sim = 0:0.01:15;
x_initial= [0 0 5*pi/180 0 0 0]; % Tilted slightly to see it move
T_val = 0; % No motor torque for now

% Note: We pass 'accel_function' into the ODE solver
[t0,x0] = ode45(@(t,y) smd(y, accel_function, -K*(y-x_ref)), t_sim, x_initial);

% 3. Plot
subplot(2,3,1)
plot(t0,x0(:,1)); title('X Position');
subplot(2,3,4)
plot(t0,x0(:,2)); title('X vel');
subplot(2,3,2)
plot(t0,x0(:,3)); title('Theta (Angle)');
subplot(2,3,5)
plot(t0,x0(:,4)); title('omega_vel');
subplot(2,3,3)
plot(t0,x0(:,5)); title('s distance');
subplot(2,3,6)
plot(t0,x0(:,6)); title(' s_vel');


% 4. The Function (Must be at the end)
function dx = smd(x_state, my_accel_fun, T_val)
    % Unpack the current state (numbers, not symbols)
    % x_state = [x, vx, theta, vtheta, s, vs]
    
    % Create the input vector exactly how 'calc_vars' was defined:
    % [x, vx, theta, vtheta, s, vs, T]
    inputs = [x_state(1), x_state(2), x_state(3), x_state(4), x_state(5), x_state(6), T_val];
    
    % Get accelerations (FAST because we use the generated function)
    [ax_val, atheta_val, as_val] = my_accel_fun(inputs);
    
    % Map to output
    dx = zeros(6,1);
    dx(1) = x_state(2); % velocity x
    dx(2) = ax_val;     % accel x
    dx(3) = x_state(4); % velocity theta
    dx(4) = atheta_val; % accel theta
    dx(5) = x_state(6); % velocity s
    dx(6) = as_val;     % accel s
end


sys = ss(A_real,B_real,C,D);
G_tf = tf(sys);

matlabFunction(f_ax_lin, f_atheta_lin, f_as_lin, ...
    'File', 'get_accell', ...
    'Vars', {state_vars});

system_poles = eig(A_real-B_real*K);

observer_poles = 3*(system_poles);

C_sens =[1 0 0 0 0 0 ; 0 0 1 0 0 0];


Ke = place(A_real',C_sens',observer_poles)';


idx_m = [1,3];
idx_u = [2,4,5,6];

Aaa = A_real(idx_m, idx_m);
Aab = A_real(idx_m,idx_u);
Aba = A_real(idx_u,idx_m);
Abb = A_real(idx_u,idx_u);

Ba = B_real(idx_m, :);
Bb = B_real(idx_u, :);
observer_poles_min = 3*[-6.7908;-0.9326;-0.0167;-0.0167];

Ke_obs = place(Abb',Aab',observer_poles_min)';
% 
% D_obs = Auu - (Ke_obs * Amu);
% E_obs = Bu - (Ke_obs * Bm);
% G_obs = Aum - (Ke_obs * Amm) + (D_obs *Ke_obs );

% 5. BUILD RECONSTRUCTION MATRICES (C_hat, D_hat)
% This maps z and y back to the full 6-state vector.
% x_hat = C_hat*z + D_hat*y

% Initialize with Zeros
C_hat = zeros(6, 4);  % 6 states output, 4 states input (z)
D_hat = zeros(6, 2);  % 6 states output, 2 states input (y)

% --- FILLING THE MATRICES ROW-BY-ROW ---

% Row 1: Position x (Measured)
% x_hat(1) = y(1)
C_hat(1,:) = [0 0 0 0];
D_hat(1,:) = [1 0];     

% Row 2: Velocity vx (Estimated)
% x_hat(2) = z(1) + L_red(1,:)*y
C_hat(2,:) = [1 0 0 0];
D_hat(2,:) = Ke_obs(1,:); 

% Row 3: Angle theta (Measured)
% x_hat(3) = y(2)
C_hat(3,:) = [0 0 0 0];
D_hat(3,:) = [0 1];

% Row 4: Angular Vel vtheta (Estimated)
% x_hat(4) = z(2) + L_red(2,:)*y
C_hat(4,:) = [0 1 0 0];
D_hat(4,:) = Ke_obs(2,:);

% Row 5: Ball Pos s (Estimated)
% x_hat(5) = z(3) + L_red(3,:)*y
C_hat(5,:) = [0 0 1 0];
D_hat(5,:) = Ke_obs(3,:);

% Row 6: Ball Vel vs (Estimated)
% x_hat(6) = z(4) + L_red(4,:)*y
C_hat(6,:) = [0 0 0 1];
D_hat(6,:) = Ke_obs(4,:);

A_hat = Abb- Ke_obs*Aab;
B_hat = A_hat*Ke_obs + Aba - Ke_obs*Aaa;
F_hat = Bb -Ke_obs*Ba;
