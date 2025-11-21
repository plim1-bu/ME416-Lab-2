function [x_hat_new, P_new] = kalmanFilter(x_hat_prev, P_prev, u_k, z_k, dt, Q, R)
% KALMANFILTER - Extended Kalman Filter for Unicycle Model
% 
% INPUTS:
%   x_hat_prev - Previous state estimate [x; y; theta]
%   P_prev     - Previous covariance matrix (3x3)
%   u_k        - Control input [v; omega]
%   z_k        - Measurement [x_mocap; y_mocap; theta_mocap]
%   dt         - Time step [s]
%   Q          - Process Noise Covariance (3x3)
%   R          - Measurement Noise Covariance (3x3)
%
% OUTPUTS:
%   x_hat_new  - Updated state estimate
%   P_new      - Updated covariance matrix

    % Extract previous state
    x_prev = x_hat_prev(1);
    y_prev = x_hat_prev(2);
    theta_prev = x_hat_prev(3);

    % Extract control inputs
    v = u_k(1);         % Linear velocity
    omega = u_k(2);     % Angular velocity
    
    %% --- 1. PREDICTION STEP (Time Update) ---
    
    % Predict State using Unicycle Kinematics
    % x' = x + v*cos(theta)*dt
    % y' = y + v*sin(theta)*dt
    % theta' = theta + omega*dt
    
    % Half-step integration for better accuracy on curves
    theta_half = theta_prev + omega * dt / 2;
    
    x_minus = x_prev + v * dt * cos(theta_half);
    y_minus = y_prev + v * dt * sin(theta_half);
    theta_minus = theta_prev + omega * dt;
    
    x_hat_minus = [x_minus; y_minus; theta_minus];
    
    % Calculate Jacobian Fk (df/dx)
    % Linear approximation of how error propagates
    Fk = [ 1, 0, -v * dt * sin(theta_half);
           0, 1,  v * dt * cos(theta_half);
           0, 0,  1 ];
           
    % Predict Covariance
    P_minus = Fk * P_prev * Fk' + Q;
    
    
    %% --- 2. UPDATE STEP (Measurement Update) ---
    
    % Measurement Matrix H (We measure [x, y, theta] directly)
    H = eye(3); 
    
    % Calculate Measurement Residual (Innovation)
    % yk = z_k - H * x_hat_minus
    yk = z_k - x_hat_minus;
    
    % --- CRITICAL FIX: ANGLE WRAPPING ---
    % Normalize the angle error to be between [-pi, pi]
    % We use atan2(sin, cos) instead of wrapToPi to avoid Toolbox errors
    yk(3) = atan2(sin(yk(3)), cos(yk(3)));
    
    % Innovation Covariance S
    S = H * P_minus * H' + R;
    
    % Kalman Gain K
    K = (P_minus * H') / S;
    
    % Update State Estimate
    x_hat_new = x_hat_minus + K * yk;
    
    % --- ANGLE WRAPPING AGAIN ---
    % Ensure final heading is also normalized
    x_hat_new(3) = atan2(sin(x_hat_new(3)), cos(x_hat_new(3)));
    
    % Update Covariance Matrix
    P_new = (eye(3) - K * H) * P_minus;
    
end
