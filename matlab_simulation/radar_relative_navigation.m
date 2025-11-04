% radar_ekf_mission_accurate.m
% Mission-accurate radar EKF for relative navigation (meters, m/s)
% State: [x; y; vx; vy] in LVLH (chaser wrt target)
% Measurements: range (m), range-rate (m/s) from radar mounted on chaser
% Author: (You) - use in research paper

clear; close all; clc;

%% ----------------- Mission / Orbit parameters -----------------
mu = 3.986004418e14;       % Earth's mu [m^3/s^2]
Re = 6371e3;               % Earth radius [m]
h = 700e3;                 % target altitude 700 km
a = Re + h;                % circular orbit radius [m]
n = sqrt(mu / a^3);       % mean motion [rad/s]

%% ----------------- Simulation parameters ---------------------
dt = 1.0;                  % EKF timestep [s]
Tsim = 3600;               % total sim time [s] (e.g., 1 hour)
N = floor(Tsim/dt);

% Process & measurement noise tuning (tune these for your radar/IMU)
% Process noise (model errors: unmodelled accel, disturbance) - continuous approx
sigma_acc = 1e-4;          % m/s^2 typical small disturbance (ion-thrust errors etc.)
Qc = diag([0.5*dt^2, 0.5*dt^2, dt, dt]) * sigma_acc^2; % heuristic continuous->discrete approx
Q = blkdiag( (dt^3/3)*sigma_acc^2*eye(2), (dt)*sigma_acc^2*eye(2) ); % 4x4 approx

% Measurement noise - radar quality
sigma_r = 1.0;             % range noise [m] (good space radar)
sigma_rr = 0.01;           % range-rate noise [m/s] (Doppler)
R = diag([sigma_r^2, sigma_rr^2]);

% Initial true state (close proximity scenario, meters)
x_true = [200;  0; 0; 0];  % 200 m radial offset (x) in LVLH, zero relative vel
% Small along-track offset variant could be used too.

% Initial estimate and covariance (realistic initial uncertainty)
x_est = x_true + [10; -5; 0.05; -0.02];   % small bias in initial estimate
P = diag([50^2, 50^2, 0.5^2, 0.5^2]);     % 50 m pos, 0.5 m/s vel uncertainty

% Store histories
Xtrue = zeros(4,N);
Xest  = zeros(4,N);
Pdiag = zeros(4,N);
zstore = zeros(2,N);

%% Precompute continuous-time A and discrete-time F (linearized CWH)
A = [0 0 1 0;
     0 0 0 1;
     3*n^2 0 0 2*n;
     0 0 -2*n 0];
Phi = eye(4) + A*dt + 0.5*(A*dt)^2;  % second-order approx of matrix exponential

%% Loop
for k = 1:N
    % ---- True dynamics integration (use simple RK2 for truth to keep stable) ----
    xdot_true = A * x_true;
    x_true = x_true + xdot_true * dt;     % simple Euler (small dt)
    
    % ---- Radar measurement (nonlinear) ----
    rx = x_true(1); ry = x_true(2); vx = x_true(3); vy = x_true(4);
    range = sqrt(rx^2 + ry^2);
    rr = (rx*vx + ry*vy) / max(range,1e-8);   % avoid div0
    z = [ range + sigma_r*randn; rr + sigma_rr*randn ];
    
    % ---- EKF Prediction ----
    x_pred = Phi * x_est;
    P_pred = Phi * P * Phi' + Q;
    
    % ---- Measurement prediction and Jacobian H ----
    px = x_pred(1); py = x_pred(2); pvx = x_pred(3); pvy = x_pred(4);
    prange = sqrt(px^2 + py^2);
    prank = max(prange,1e-8);
    z_pred = [prank;
              (px*pvx + py*pvy)/prank];
          
    % Jacobian H (2x4)
    H = zeros(2,4);
    H(1,1) = px/prank;
    H(1,2) = py/prank;
    H(2,1) = (pvx*prank - px*(px*pvx + py*pvy)/prank) / (prank^2);
    H(2,2) = (pvy*prank - py*(px*pvx + py*pvy)/prank) / (prank^2);
    H(2,3) = px/prank;
    H(2,4) = py/prank;
    
    % ---- Kalman Gain & Update ----
    S = H * P_pred * H' + R;
    K = (P_pred * H') / S;
    x_est = x_pred + K * (z - z_pred);
    P = (eye(4) - K*H) * P_pred;
    
    % ---- Log ----
    Xtrue(:,k) = x_true;
    Xest(:,k)  = x_est;
    Pdiag(:,k) = diag(P);
    zstore(:,k) = z;
end

%% ------------------ Plots -------------------------------
t = (0:N-1)*dt/60;   % minutes

% Position: true vs estimated
figure('Name','EKF Position Estimates','Color','w','Units','normalized','Position',[0.05 0.05 0.6 0.7]);
subplot(3,1,1);
plot(t, Xtrue(1,:)/1e0,'b','LineWidth',1.5); hold on;
plot(t, Xest(1,:)/1e0,'--r','LineWidth',1.2);
ylabel('x [m]'); grid on; legend('True','Estimated');
title('Relative Position & EKF Performance');

subplot(3,1,2);
plot(t, Xtrue(2,:)/1e0,'b','LineWidth',1.5); hold on;
plot(t, Xest(2,:)/1e0,'--r','LineWidth',1.2);
ylabel('y [m]'); grid on; legend('True','Estimated');

% Range measurement and residual
subplot(3,1,3);
plot(t, zstore(1,:)/1e0,'k'); hold on;
plot(t, sqrt(Xest(1,:).^2 + Xest(2,:).^2)/1e0,'--r');
ylabel('Range [m]'); xlabel('Time [min]'); grid on; legend('Measurement','Estimated Range');

% Estimation error + 3-sigma envelope
figure('Name','EKF Errors & Covariance','Color','w','Units','normalized','Position',[0.65 0.05 0.3 0.7]);
err = Xest - Xtrue;
clf;
subplot(4,1,1); hold on; grid on;
plot(t, err(1,:),'b'); plot(t, 3*sqrt(Pdiag(1,:)),'r--'); plot(t, -3*sqrt(Pdiag(1,:)),'r--');
ylabel('x error [m]'); legend('error','\pm3\sigma');

subplot(4,1,2); hold on; grid on;
plot(t, err(2,:),'b'); plot(t, 3*sqrt(Pdiag(2,:)),'r--'); plot(t, -3*sqrt(Pdiag(2,:)),'r--');
ylabel('y error [m]');

subplot(4,1,3); hold on; grid on;
plot(t, err(3,:),'b'); plot(t, 3*sqrt(Pdiag(3,:)),'r--'); plot(t, -3*sqrt(Pdiag(3,:)),'r--');
ylabel('vx err [m/s]');

subplot(4,1,4); hold on; grid on;
plot(t, err(4,:),'b'); plot(t, 3*sqrt(Pdiag(4,:)),'r--'); plot(t, -3*sqrt(Pdiag(4,:)),'r--');
ylabel('vy err [m/s]'); xlabel('Time [min]');

%% ------------------ Summary Stats -------------------------
pos_rms = sqrt(mean(err(1,:).^2 + err(2,:).^2));
vel_rms = sqrt(mean(err(3,:).^2 + err(4,:).^2));
fprintf('Position RMS error: %.3f m\n', pos_rms);
fprintf('Velocity RMS error: %.4f m/s\n', vel_rms);

% Save outputs (optional)
% save('radar_ekf_results.mat','Xtrue','Xest','Pdiag','zstore','dt','n');

