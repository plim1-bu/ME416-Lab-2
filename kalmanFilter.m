function [x_hat_new, P_new] = kalmanFilter(x_hat_prev, P_prev, u_k, z_k, dt, Q, R)
% x_hat_prev: [x; y; theta] previous state estimate
% P_prev: 3x3 covariance matrix from previous step
% u_k: [v; omega] control inputs from the last cycle
% z_k: [x_mocap; y_mocap; theta_mocap] current MoCap measurement
% dt: Time step
% Q: Process Noise Covariance matrix (3x3)
% R: Measurement Noise Covariance matrix (3x3)

    % Extract previous state
    x_prev = x_hat_prev(1);
    y_prev = x_hat_prev(2);
    theta_prev = x_hat_prev(3);

    % Extract control inputs
    v = u_k(1);         % Linear velocity = u_k(1) = v
    omega = u_k(2);     % Angular velocity = u_k(2) = omega
    
    %% --- 1. PREDICTION STEP ---
    
    % Predicted State (x_hat_minus)
    x_minus = x_prev + v*dt * cos(theta_prev + omega*dt/2);
    y_minus = y_prev + v*dt * sin(theta_prev + omega*dt/2);
    theta_minus = theta_prev + omega*dt;
    
    x_hat_minus = [x_minus; y_minus; theta_minus];
    
    % Calculate the Jacobian Fk = df/dx
    Fk = [ 1, 0, -v*dt*sin(theta_prev + omega*dt/2);
           0, 1,  v*dt*cos(theta_prev + omega*dt/2);
           0, 0,  1 ];
    % Predicted Covariance (P_minus)
    P_minus = Fk * P_prev * Fk' + Q;
    
    
    %% --- 2. UPDATE STEP ---
    
    % Measurement Matrix H: h(x) is identity, so H = dh/dx is the 3x3 identity matrix
    H = eye(3); 
    
    % Innovation (Residual) Covariance S
    S = H * P_minus * H' + R;
    
    % Kalman Gain K
    K = (P_minus * H') / S;
    
    % Innovation (Residual) yk
    yk = z_k - x_hat_minus;
    
    % Wrap theta in (-pi, pi]
    while yk(3) > pi
        yk(3) = yk(3) - 2*pi;
    end
    while yk(3) <= -pi
        yk(3) = yk(3) + 2*pi;
    end
    
    % Updated State Estimate (x_hat_new)
    x_hat_new = x_hat_minus + K * yk;
    
    % Wrap orientation estimate in (-pi, pi]
    while x_hat_new(3) > pi
        x_hat_new(3) = x_hat_new(3) - 2*pi;
    end
    while x_hat_new(3) <= -pi
        x_hat_new(3) = x_hat_new(3) + 2*pi;
    end
    
    % Updated Covariance (P_new)
    P_new = (eye(3) - K * H) * P_minus;
    
end