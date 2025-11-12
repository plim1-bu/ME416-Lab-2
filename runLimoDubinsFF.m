function runLimoDubinsFF()
% RUNLIMODUBINSFF - Feedforward control for Limo using TCP
% Uses dubinsConnection with robust interpolation and error handling.

%% ---------------- 1. Configuration Constants ----------------
CONST.V_MAX = 0.3;                  % Max linear velocity [m/s]
CONST.DT = 0.55;                    % Feedforward interval [s] as suggested by lab manual
CONST.R_MIN = 0.6;                  % Minimum turning radius [m]. INCREASED to 0.6m to force a gentler curve, reducing slip/positional 
CONST.K_VELOCITY = 0.96;            % Linear Velocity Gain (K_V). CALIBRATED: Adjusted post overshoots in testing
CONST.K_OMEGA = 1.6;                % Angular Velocity Gain (K_omega). CALIBRATED: Adjusted post overshoots in testing
CONST.FIXED_IP = '192.168.1.';      % First 3 octets of Limo IP
CONST.LIMO_PORT = 12345;
CONST.FINAL_ROTATION_STEPS = 15;    % Buffer steps at end of runtime for final rotation
CONST.INTERPOLATION_POINTS = 1000;  % High number for robust path length estimation

% Initialize TCP client handle
tcpClient = []; 

try
    %% ---------------- 2. User Input ----------------

    % x axis is forward positive, y axis is left positive, theta is counterclockwise positive
    disp('Enter start pose [x, y, theta] in meters and degrees:');
    x0 = input('Start x (m): ');
    y0 = input('Start y (m): ');
    theta0_deg = input('Start theta (deg): ');
    
    disp('Enter goal pose [x, y, theta] in meters and degrees:');
    xf = input('Goal x (m): ');
    yf = input('Goal y (m): ');
    thetaf_deg = input('Goal theta (deg): ');
    
    lastDigits = input('Enter last 3 digits of Limo IP: ','s');
    
    % Convert degrees to radians and normalize goal angle
    theta0 = deg2rad(theta0_deg);
    thetaf_rad = deg2rad(thetaf_deg);
    thetaf = wrapToPi(thetaf_rad); % Robust normalization
    
    fprintf('Goal theta normalized from %.4f rad to %.4f rad for planning.\n', thetaf_rad, thetaf);
    
    startPose = [x0, y0, theta0];
    goalPose  = [xf, yf, thetaf];
    limoIP = [CONST.FIXED_IP, lastDigits];

    %% ---------------- 3. Path Planning ----------------
    [path_points, num_steps] = dubinsPathPlanning(startPose, goalPose, CONST);
    if isempty(path_points)
        disp('Script stopped due to path planning failure.');
        return;
    end

    %% ---------------- 4. TCP Connection ----------------
    tcpClient = tcpConnection(limoIP, CONST.LIMO_PORT);
    if isempty(tcpClient)
        disp('Script stopped due to connection failure.');
        return;
    end

    % Extract path coordinates and orientation
    path_x = path_points(:, 1);
    path_y = path_points(:, 2);
    path_theta = path_points(:, 3);
    
    % Initial state log
    fprintf('t = %.2f s | x_d = %.2f, y_d = %.2f, theta_d = %.2f | v = %.2f, omega = %.2f\n', ...
            0, path_x(1), path_y(1), path_theta(1), 0, 0); 

    %% ---------------- 5. Feedforward Loop ----------------
    % Iterates through each time step
    for k = 1:num_steps - 1
       % Current time of execution
        t = k * CONST.DT;
        
        % Current desired pose
        x_d = path_x(k);
        y_d = path_y(k);
        theta_d = path_theta(k);
        
        % Next desired pose (for finite difference calculation)
        x_next = path_x(k+1);
        y_next = path_y(k+1);
        theta_next = path_theta(k+1);
        
        sendFeedforwardCommand(tcpClient, x_d, y_d, theta_d, x_next, y_next, theta_next, CONST, t);

        % Wait for specified time step
        pause(CONST.DT);
    end

catch ME
    % Catch any errors that happen during execution (e.g., in the loop)
    warning('An error occurred during execution: %s', ME.message);
     
    %% ---------------- 6. Cleanup and Stop Command ----------------
    if ~isempty(tcpClient) && isvalid(tcpClient)
        % Send final stop command (k=num_steps)
        cmd = sprintf('0.00,0.00');
        write(tcpClient, uint8(cmd));
        
        % PAUSE added to give Limo time to zero its steering angle
        pause(1.0); 
        
        clear tcpClient; % Closes the connection
        disp('Feedforward path complete. TCP connection closed.');
    else
        disp('No active TCP connection was established or needed closing.');
    end
end

disp('Script execution finished.');
end % end of runLimoDubinsFF

%% ---------------- Local Functions ----------------
function [path_points, num_steps] = dubinsPathPlanning(startPose, goalPose, CONST)
    % DUBINSPATHPLANNING Calculates the path points and number of steps.

    % Create Dubins path object
    dub = dubinsConnection('MinTurningRadius', CONST.R_MIN);
    path_points = []; % Initialize
    num_steps = 0; % Initialize
    
    try
        dubPath_raw = connect(dub, startPose, goalPose);                                  % Calculate shortest Dubins curve sequence.
        
        if isempty(dubPath_raw) || ~iscell(dubPath_raw)
            error('Dubins path failed. Check R_min or pose configuration.');
        end
        
        dubPath = dubPath_raw{1};                                                         % Unwraps from cell array to path object.
        
        % --- Robust Length Calculation ---
        % Interpolate many points and sum the Euclidean distance between them.
        temp_points = interpolate(dubPath, linspace(0, 1, CONST.INTERPOLATION_POINTS));   % Interpolate path
        path_length = sum(vecnorm(diff(temp_points(:, 1:2)), 2, 2));                      % Calculate total length by summing interpolated points
        
        % --- Determine Step Count and Final Interpolation ---
        % Calculate required steps based on V_MAX and DT, adding a buffer.
        base_steps = ceil(path_length / (CONST.V_MAX * CONST.DT));                        % Minimum number of steps based on path length and maximum speed.
        num_steps = base_steps + CONST.FINAL_ROTATION_STEPS;                              % Add buffer steps for final rotation steps.
        
        % Create distance vector for final interpolation
        path_s = linspace(0, path_length, num_steps);
        
        % Interpolate path_points based on number of steps to ensure the robot travels at a consistent pace
        path_points = interpolate(dubPath, path_s);
        
        disp('Dubins path generated successfully using robust length calculation.');
        
        % --- Path Visualization (Debug) ---
        figure('Name', 'Dubins Path Visualization');
        hold on;
        plot(path_points(:, 1), path_points(:, 2), 'b-', 'LineWidth', 2);
        quiver(startPose(1), startPose(2), cos(startPose(3)), sin(startPose(3)), 0.5, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        plot(startPose(1), startPose(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        quiver(goalPose(1), goalPose(2), cos(goalPose(3)), sin(goalPose(3)), 0.5, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        plot(goalPose(1), goalPose(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        title('Planned Dubins Path');
        xlabel('X (m)');
        ylabel('Y (m)');
        axis equal;
        grid on;
        hold off;

    catch ME
        % Display only a warning here so the main function can handle the cleanup
        warning('Path planning failed: %s', ME.message);
    end
end

function tcpClient = tcpConnection(limoIP, limoPort)
    % TCPCONNECTION Establishes the TCP connection.
    tcpClient = [];   % Initialize to empty array
    try
        % Using a 5-second timeout for connection establishment
        tcpClient = tcpclient(limoIP, limoPort, 'Timeout', 5); 
        disp(['Connected to Limo ', limoIP, ' via TCP/IP.']);
    catch ME
        fprintf('Warning: Attempted to connect to: %s:%d\n', limoIP, limoPort);
        % Throw an error to stop the main script
        error('Unable to connect. Check IP, WiFi, and TCP server running on Limo. Underlying error: %s', ME.message);
    end
end

function sendFeedforwardCommand(tcpClient, x_d, y_d, theta_d, x_next, y_next, theta_next, CONST, t)
    % SENDFEEDFORWARDCOMMAND Computes and sends one feedforward command.
    
    % Compute linear velocity (finite difference for position)
    dx = (x_next - x_d);                 % Change in x
    dy = (y_next - y_d);                 % Change in y
    v_d = sqrt(dx^2 + dy^2) / CONST.DT;  % Total displacement calculation
    
    % Compute angular velocity (finite difference for orientation)
    dtheta = angdiff(theta_d, theta_next);     % Change in angle
    omega_raw = dtheta / CONST.DT;             % RAW angular velocity before gain
    
    % Apply LINEAR Gain
    v_d = v_d * CONST.K_VELOCITY; 
    
    % Apply ANGULAR Gain
    omega_sent = omega_raw * CONST.K_OMEGA;

    % Caps linear velocity to V_MAX
    v_d = min(v_d, CONST.V_MAX);
    
    % Send command over TCP (V_d, Omega_d)
    cmd = sprintf('%.2f,%.2f', v_d, omega_sent);
    write(tcpClient, uint8(cmd));
    
    % Display diagnostic info
    fprintf('t=%.2f s | v=%.2f, omega_raw=%.2f, omega_sent=%.2f\n', ...
        t, v_d, omega_raw, omega_sent);
end
