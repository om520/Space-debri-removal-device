% end_to_end_ion_ekf.m
% End-to-end: low-thrust chaser truth -> LVLH relative -> Radar meas -> EKF
% Units: meters, seconds
clear; close all; clc;

%% --------------------- Mission / Orbit params ------------------------
mu = 3.986004418e14;        % [m^3/s^2]
Re = 6371e3;                % [m]
h_target = 700e3;           % target altitude 700 km
a_target = Re + h_target;   % orbital radius (circular)
n = sqrt(mu / a_target^3);  % mean motion

%% --------------------- Simulation params ----------------------------
dt = 1;                     % time step [s]
Tsim = 3600;                % total sim time [s]
N = Tsim/dt;

%% --------------------- Spacecraft initial states (ECI planar) -------
% Target: circular orbit in xy-plane, initial position at x = a_target, y=0
rT0 = [a_target; 0; 0];                 % [m]
vT0 = [0; sqrt(mu/a_target); 0];        % [m/s]

% Chaser: small LVLH offset relative to target (e.g., 200 m radial outward, -10 m along-track)
% Build initial LVLH basis for target
rhat0 = rT0 / norm(rT0);
hvec0 = cross(rT0, vT0);
hhat0 = hvec0 / norm(hvec0);
that0 = cross(hhat0, rhat0);

% Relative LVLH offset
x_rel0 = 200;       % radial offset [m]
y_rel0 = -10;       % along-track offset [m]
z_rel0 = 0;         % out-of-plane [m]
% Convert LVLH->ECI for chaser initial r and v
rC0 = rT0 + x_rel0 * rhat0 + y_rel0 * that0 + z_rel0 * hhat0;
% initial relative velocity approx using CWH: v_rel ≈ [-n*y, 2*n*x] small
vx_rel0 = -n * y_rel0;
vy_rel0 =  2*n * x_rel0;
vC0 = vT0 + vx_rel0 * rhat0 + vy_rel0 * that0;

% Pack initial states
stateT = [rT0; vT0];       % target state vector [rx;ry;rz;vx;vy;vz]
stateC = [rC0; vC0];       % chaser state vector

%% --------------------- Thruster model (chaser) ----------------------
T_newton = 0.3;            % thrust [N] (ion thruster)
Isp = 3000;                % s
mass0 = 250;               % kg
m = mass0;
% For simplicity we model thrust acceleration in direction of chaser vel unit vector
% (tangential). Mass flow will be computed but not used to change m significantly over Tsim.

mdot = -(T_newton)/(Isp*9.80665); % kg/s

%% --------------------- EKF params (LVLH) -----------------------------
% Process noise: small unmodeled acceleration (m/s^2)
sigma_acc = 2e-5; % tuneable
Q = blkdiag((dt^3/3)*sigma_acc^2*eye(2), (dt)*sigma_acc^2*eye(2));
% Measurement noise
sigma_r = 1.0;     % range noise [m]
sigma_rr = 0.01;   % range-rate noise [m/s]
R = diag([sigma_r^2, sigma_rr^2]);

% Initial EKF estimate: small bias from true relative
x_true0 = [x_rel0; y_rel0; vx_rel0; vy_rel0];
x_est = x_true0 + [10; -5; 0.02; -0.01];  % initial estimate (4x1)
P = diag([50^2, 50^2, 0.5^2, 0.5^2]);

% State and log storage
Xtrue_rel = zeros(4,N);
Xest = zeros(4,N);
Pdiag = zeros(4,N);
zlog = zeros(2,N);
tvec = (0:N-1)*dt;

%% --------------------- Integration loop (RK4 for ECI truth) ----------
for k = 1:N
    % --- propagate target (2-body) using RK4 ---
    stateT = rk4_step(@twobody, stateT, dt, mu, 0);  % no thrust on target
    % --- propagate chaser with small tangential thrust ---
    % compute thrust accel in ECI (tangential = along current velocity)
    vC = stateC(4:6);
    speedC = norm(vC);
    if speedC > 1e-8
        u_tang = vC / speedC;
    else
        u_tang = [0;0;0];
    end
    a_thrust = (T_newton / m) * u_tang; % m/s^2
    stateC = rk4_step(@twobody, stateC, dt, mu, a_thrust); % include thrust as accel input
    % update mass (simple)
    m = m + mdot * dt; % mdot is negative
    
    % --- compute relative vector in ECI and LVLH basis (target-centered) ---
    rT = stateT(1:3); vT = stateT(4:6);
    rC = stateC(1:3); vC = stateC(4:6);
    r_rel = rC - rT;           % ECI relative vector
    v_rel = vC - vT;
    % compute LVLH basis at target
    rhat = rT / norm(rT);
    hvec = cross(rT, vT);
    hhat = hvec / norm(hvec);
    that = cross(hhat, rhat);
    % LVLH coordinates (radial, along-track, cross-track)
    x_rel = dot(r_rel, rhat);
    y_rel = dot(r_rel, that);
    z_rel = dot(r_rel, hhat);
    vx_rel = dot(v_rel, rhat);
    vy_rel = dot(v_rel, that);
    vz_rel = dot(v_rel, hhat);
    % Use 2D (radial/along-track) state for EKF
    x_true = [x_rel; y_rel; vx_rel; vy_rel];
    
    % --- synthesize radar measurement (range, range-rate) with noise ---
    range = sqrt(x_rel^2 + y_rel^2 + z_rel^2);
    range_rate = (dot(r_rel, v_rel))/max(range,1e-8);
    z = [range + sigma_r*randn; range_rate + sigma_rr*randn];
    
    % --- EKF prediction (discrete CWH linearization about target orbit) ---
    % Build CWH A matrix using n computed from target radius (recompute for varying rT if desired)
    rnorm = norm(rT);
    n_local = sqrt(mu / rnorm^3);
    A = [0 0 1 0;
         0 0 0 1;
         3*n_local^2 0 0 2*n_local;
         0 0 -2*n_local 0];
    Phi = eye(4) + A*dt + 0.5*(A*dt)^2; % 2nd order approx
    x_pred = Phi * x_est;
    P_pred = Phi * P * Phi' + Q;
    
    % --- measurement prediction and Jacobian (nonlinear) ---
    px = x_pred(1); py = x_pred(2); pvx = x_pred(3); pvy = x_pred(4);
    prange = sqrt(px^2 + py^2 + 0^2);
    prange = max(prange, 1e-8);
    z_pred = [prange;
              (px*pvx + py*pvy)/prange];
    % Jacobian H (2x4)
    H = zeros(2,4);
    H(1,1) = px/prange; H(1,2) = py/prange;
    H(2,1) = (pvx*prange - px*(px*pvx + py*pvy)/prange) / (prange^2);
    H(2,2) = (pvy*prange - py*(px*pvx + py*pvy)/prange) / (prange^2);
    H(2,3) = px/prange; H(2,4) = py/prange;
    
    % --- update step ---
    S = H * P_pred * H' + R;
    K = (P_pred * H') / S;
    x_est = x_pred + K * (z - z_pred);
    P = (eye(4) - K*H) * P_pred;
    
    % --- store logs ---
    Xtrue_rel(:,k) = x_true;
    Xest(:,k) = x_est;
    Pdiag(:,k) = diag(P);
    zlog(:,k) = z;
end

%% --------------------- Plots & Stats --------------------------------
tmin = tvec/60;
% Position & range plots
figure('Name','End-to-End: LVLH Rel Nav','Color','w','Position',[50 50 900 700]);
subplot(3,1,1);
plot(tmin, Xtrue_rel(1,:),'b','LineWidth',1.3); hold on;
plot(tmin, Xest(1,:),'--r','LineWidth',1.2); ylabel('x [m]'); grid on; legend('True','Est');
title('LVLH Relative Position & EKF');

subplot(3,1,2);
plot(tmin, Xtrue_rel(2,:),'b','LineWidth',1.3); hold on;
plot(tmin, Xest(2,:),'--r','LineWidth',1.2); ylabel('y [m]'); grid on; legend('True','Est');

subplot(3,1,3);
plot(tmin, zlog(1,:),'k','LineWidth',1); hold on;
plot(tmin, sqrt(Xest(1,:).^2 + Xest(2,:).^2),'--r'); ylabel('Range [m]'); xlabel('Time [min]'); grid on; legend('Meas','EstRange');

% Error and 3-sigma
figure('Name','Errors & 3-sigma','Color','w','Position',[980 50 420 700]);
err = Xest - Xtrue_rel;
subplot(4,1,1); hold on; grid on;
plot(tmin, err(1,:),'b'); plot(tmin, 3*sqrt(Pdiag(1,:)),'r--'); plot(tmin, -3*sqrt(Pdiag(1,:)),'r--');
ylabel('x err [m]'); legend('err','\pm3\sigma');

subplot(4,1,2); hold on; grid on;
plot(tmin, err(2,:),'b'); plot(tmin, 3*sqrt(Pdiag(2,:)),'r--'); plot(tmin, -3*sqrt(Pdiag(2,:)),'r--');
ylabel('y err [m]');

subplot(4,1,3); hold on; grid on;
plot(tmin, err(3,:),'b'); plot(tmin, 3*sqrt(Pdiag(3,:)),'r--'); plot(tmin, -3*sqrt(Pdiag(3,:)),'r--');
ylabel('vx err [m/s]');

subplot(4,1,4); hold on; grid on;
plot(tmin, err(4,:),'b'); plot(tmin, 3*sqrt(Pdiag(4,:)),'r--'); plot(tmin, -3*sqrt(Pdiag(4,:)),'r--');
ylabel('vy err [m/s]'); xlabel('Time [min]');

% Print RMS
pos_rms = sqrt(mean(err(1,:).^2 + err(2,:).^2));
vel_rms = sqrt(mean(err(3,:).^2 + err(4,:).^2));
fprintf('End-to-end Position RMS error: %.3f m\n', pos_rms);
fprintf('End-to-end Velocity RMS error: %.4f m/s\n', vel_rms);

%% --------------------- Helper functions -----------------------------
function dst = twobody(~, state, mu, extra_accel)
    % state: [rx;ry;rz;vx;vy;vz]
    r = state(1:3% end_to_end_ion_ekf.m
% End-to-end: low-thrust chaser truth -> LVLH relative -> Radar meas -> EKF
% Units: meters, seconds
clear; close all; clc;

%% --------------------- Mission / Orbit params ------------------------
mu = 3.986004418e14;        % [m^3/s^2]
Re = 6371e3;                % [m]
h_target = 700e3;           % target altitude 700 km
a_target = Re + h_target;   % orbital radius (circular)
n = sqrt(mu / a_target^3);  % mean motion

%% --------------------- Simulation params ----------------------------
dt = 1;                     % time step [s]
Tsim = 3600;                % total sim time [s]
N = Tsim/dt;

%% --------------------- Spacecraft initial states (ECI planar) -------
% Target: circular orbit in xy-plane, initial position at x = a_target, y=0
rT0 = [a_target; 0; 0];                 % [m]
vT0 = [0; sqrt(mu/a_target); 0];        % [m/s]

% Chaser: small LVLH offset relative to target (e.g., 200 m radial outward, -10 m along-track)
% Build initial LVLH basis for target
rhat0 = rT0 / norm(rT0);
hvec0 = cross(rT0, vT0);
hhat0 = hvec0 / norm(hvec0);
that0 = cross(hhat0, rhat0);

% Relative LVLH offset
x_rel0 = 200;       % radial offset [m]
y_rel0 = -10;       % along-track offset [m]
z_rel0 = 0;         % out-of-plane [m]
% Convert LVLH->ECI for chaser initial r and v
rC0 = rT0 + x_rel0 * rhat0 + y_rel0 * that0 + z_rel0 * hhat0;
% initial relative velocity approx using CWH: v_rel ≈ [-n*y, 2*n*x] small
vx_rel0 = -n * y_rel0;
vy_rel0 =  2*n * x_rel0;
vC0 = vT0 + vx_rel0 * rhat0 + vy_rel0 * that0;

% Pack initial states
stateT = [rT0; vT0];       % target state vector [rx;ry;rz;vx;vy;vz]
stateC = [rC0; vC0];       % chaser state vector

%% --------------------- Thruster model (chaser) ----------------------
T_newton = 0.3;            % thrust [N] (ion thruster)
Isp = 3000;                % s
mass0 = 250;               % kg
m = mass0;
% For simplicity we model thrust acceleration in direction of chaser vel unit vector
% (tangential). Mass flow will be computed but not used to change m significantly over Tsim.

mdot = -(T_newton)/(Isp*9.80665); % kg/s

%% --------------------- EKF params (LVLH) -----------------------------
% Process noise: small unmodeled acceleration (m/s^2)
sigma_acc = 2e-5; % tuneable
Q = blkdiag((dt^3/3)*sigma_acc^2*eye(2), (dt)*sigma_acc^2*eye(2));
% Measurement noise
sigma_r = 1.0;     % range noise [m]
sigma_rr = 0.01;   % range-rate noise [m/s]
R = diag([sigma_r^2, sigma_rr^2]);

% Initial EKF estimate: small bias from true relative
x_true0 = [x_rel0; y_rel0; vx_rel0; vy_rel0];
x_est = x_true0 + [10; -5; 0.02; -0.01];  % initial estimate (4x1)
P = diag([50^2, 50^2, 0.5^2, 0.5^2]);

% State and log storage
Xtrue_rel = zeros(4,N);
Xest = zeros(4,N);
Pdiag = zeros(4,N);
zlog = zeros(2,N);
tvec = (0:N-1)*dt;

%% --------------------- Integration loop (RK4 for ECI truth) ----------
for k = 1:N
    % --- propagate target (2-body) using RK4 ---
    stateT = rk4_step(@twobody, stateT, dt, mu, 0);  % no thrust on target
    % --- propagate chaser with small tangential thrust ---
    % compute thrust accel in ECI (tangential = along current velocity)
    vC = stateC(4:6);
    speedC = norm(vC);
    if speedC > 1e-8
        u_tang = vC / speedC;
    else
        u_tang = [0;0;0];
    end
    a_thrust = (T_newton / m) * u_tang; % m/s^2
    stateC = rk4_step(@twobody, stateC, dt, mu, a_thrust); % include thrust as accel input
    % update mass (simple)
    m = m + mdot * dt; % mdot is negative
    
    % --- compute relative vector in ECI and LVLH basis (target-centered) ---
    rT = stateT(1:3); vT = stateT(4:6);
    rC = stateC(1:3); vC = stateC(4:6);
    r_rel = rC - rT;           % ECI relative vector
    v_rel = vC - vT;
    % compute LVLH basis at target
    rhat = rT / norm(rT);
    hvec = cross(rT, vT);
    hhat = hvec / norm(hvec);
    that = cross(hhat, rhat);
    % LVLH coordinates (radial, along-track, cross-track)
    x_rel = dot(r_rel, rhat);
    y_rel = dot(r_rel, that);
    z_rel = dot(r_rel, hhat);
    vx_rel = dot(v_rel, rhat);
    vy_rel = dot(v_rel, that);
    vz_rel = dot(v_rel, hhat);
    % Use 2D (radial/along-track) state for EKF
    x_true = [x_rel; y_rel; vx_rel; vy_rel];
    
    % --- synthesize radar measurement (range, range-rate) with noise ---
    range = sqrt(x_rel^2 + y_rel^2 + z_rel^2);
    range_rate = (dot(r_rel, v_rel))/max(range,1e-8);
    z = [range + sigma_r*randn; range_rate + sigma_rr*randn];
    
    % --- EKF prediction (discrete CWH linearization about target orbit) ---
    % Build CWH A matrix using n computed from target radius (recompute for varying rT if desired)
    rnorm = norm(rT);
    n_local = sqrt(mu / rnorm^3);
    A = [0 0 1 0;
         0 0 0 1;
         3*n_local^2 0 0 2*n_local;
         0 0 -2*n_local 0];
    Phi = eye(4) + A*dt + 0.5*(A*dt)^2; % 2nd order approx
    x_pred = Phi * x_est;
    P_pred = Phi * P * Phi' + Q;
    
    % --- measurement prediction and Jacobian (nonlinear) ---
    px = x_pred(1); py = x_pred(2); pvx = x_pred(3); pvy = x_pred(4);
    prange = sqrt(px^2 + py^2 + 0^2);
    prange = max(prange, 1e-8);
    z_pred = [prange;
              (px*pvx + py*pvy)/prange];
    % Jacobian H (2x4)
    H = zeros(2,4);
    H(1,1) = px/prange; H(1,2) = py/prange;
    H(2,1) = (pvx*prange - px*(px*pvx + py*pvy)/prange) / (prange^2);
    H(2,2) = (pvy*prange - py*(px*pvx + py*pvy)/prange) / (prange^2);
    H(2,3) = px/prange; H(2,4) = py/prange;
    
    % --- update step ---
    S = H * P_pred * H' + R;
    K = (P_pred * H') / S;
    x_est = x_pred + K * (z - z_pred);
    P = (eye(4) - K*H) * P_pred;
    
    % --- store logs ---
    Xtrue_rel(:,k) = x_true;
    Xest(:,k) = x_est;
    Pdiag(:,k) = diag(P);
    zlog(:,k) = z;
end

%% --------------------- Plots & Stats --------------------------------
tmin = tvec/60;
% Position & range plots
figure('Name','End-to-End: LVLH Rel Nav','Color','w','Position',[50 50 900 700]);
subplot(3,1,1);
plot(tmin, Xtrue_rel(1,:),'b','LineWidth',1.3); hold on;
plot(tmin, Xest(1,:),'--r','LineWidth',1.2); ylabel('x [m]'); grid on; legend('True','Est');
title('LVLH Relative Position & EKF');

subplot(3,1,2);
plot(tmin, Xtrue_rel(2,:),'b','LineWidth',1.3); hold on;
plot(tmin, Xest(2,:),'--r','LineWidth',1.2); ylabel('y [m]'); grid on; legend('True','Est');

subplot(3,1,3);
plot(tmin, zlog(1,:),'k','LineWidth',1); hold on;
plot(tmin, sqrt(Xest(1,:).^2 + Xest(2,:).^2),'--r'); ylabel('Range [m]'); xlabel('Time [min]'); grid on; legend('Meas','EstRange');

% Error and 3-sigma
figure('Name','Errors & 3-sigma','Color','w','Position',[980 50 420 700]);
err = Xest - Xtrue_rel;
subplot(4,1,1); hold on; grid on;
plot(tmin, err(1,:),'b'); plot(tmin, 3*sqrt(Pdiag(1,:)),'r--'); plot(tmin, -3*sqrt(Pdiag(1,:)),'r--');
ylabel('x err [m]'); legend('err','\pm3\sigma');

subplot(4,1,2); hold on; grid on;
plot(tmin, err(2,:),'b'); plot(tmin, 3*sqrt(Pdiag(2,:)),'r--'); plot(tmin, -3*sqrt(Pdiag(2,:)),'r--');
ylabel('y err [m]');

subplot(4,1,3); hold on; grid on;
plot(tmin, err(3,:),'b'); plot(tmin, 3*sqrt(Pdiag(3,:)),'r--'); plot(tmin, -3*sqrt(Pdiag(3,:)),'r--');
ylabel('vx err [m/s]');

subplot(4,1,4); hold on; grid on;
plot(tmin, err(4,:),'b'); plot(tmin, 3*sqrt(Pdiag(4,:)),'r--'); plot(tmin, -3*sqrt(Pdiag(4,:)),'r--');
ylabel('vy err [m/s]'); xlabel('Time [min]');

% Print RMS
pos_rms = sqrt(mean(err(1,:).^2 + err(2,:).^2));
vel_rms = sqrt(mean(err(3,:).^2 + err(4,:).^2));
fprintf('End-to-end Position RMS error: %.3f m\n', pos_rms);
fprintf('End-to-end Velocity RMS error: %.4f m/s\n', vel_rms);

%% --------------------- Helper functions -----------------------------
function dst = twobody(~, state, mu, extra_accel)
    % Two-body dynamics + optional external acceleration
    % state: [rx; ry; rz; vx; vy; vz]
    r = state(1:3);
    v = state(4:6);
    rnorm = norm(r);
    a_grav = -mu * r / rnorm^3;
    if nargin < 4
        a_extra = [0; 0; 0];
    else
        a_extra = extra_accel;
    end
    dst = [v; a_grav + a_extra];
end

function state_next = rk4_step(dynfun, state, dt, mu, a_extra)
    % Fourth-order Runge-Kutta integrator with optional accel
    if nargin < 5
        a_extra = [0; 0; 0];
    end
    k1 = dynfun(0, state, mu, a_extra);
    k2 = dynfun(0, state + 0.5*dt*k1, mu, a_extra);
    k3 = dynfun(0, state + 0.5*dt*k2, mu, a_extra);
    k4 = dynfun(0, state + dt*k3, mu, a_extra);
    state_next = state + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end


function state_next = rk4_step(dynfun, state, dt, mu, a_extra)
    % Simple RK4 wrapper for twobody with optional extra accel (3x1)
    if nargin < 5, a_extra = [0;0;0]; end
    k1 = dynfun(0, state, mu, a_extra);
    k2 = dynfun(0, state + 0.5*dt*k1, mu, a_extra);
    k3 = dynfun(0, state + 0.5*dt*k2, mu, a_extra);
    k4 = dynfun(0, state + dt*k3, mu, a_extra);
    state_next = state + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end
