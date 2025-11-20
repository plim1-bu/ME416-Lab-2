function runLimoPlannerPath_MQTT()
% RUNLIMOPLANNERPATH_MQTT - Follow a predefined path using Dubins + Pure Pursuit
% Works with real LIMO hardware using MQTT MoCap feedback
% 
% This version allows selection of predefined routes (1, 2, or 3) and
% configuration of LIMO network parameters

clearvars; close all;
%% ===============================================
%% USER INPUT: ROUTE SELECTION AND LIMO CONFIGURATION
%% ===============================================
fprintf('\n========================================\n');
fprintf('  ROUTE AND LIMO CONFIGURATION\n');
fprintf('========================================\n\n');

% --- Route Selection ---
fprintf('Available Routes:\n');
fprintf('  Route 1: Two-column obstacle course with right-side path\n');
fprintf('  Route 2: 3x3 grid obstacle pattern\n');
fprintf('  Route 3: Complex S-shaped path with multiple obstacles\n\n');

% Prompt for route selection with validation
valid_route = false;
while ~valid_route
    route_input = input('Select route (1, 2, or 3): ', 's');
    route_num = str2double(route_input);
    
    % Check if input is valid (1, 2, or 3)
    if ismember(route_num, [1, 2, 3])
        valid_route = true;
    else
        fprintf('Invalid input. Please enter 1, 2, or 3.\n');
    end
end

fprintf('\n--- LIMO Network Configuration ---\n');
% --- LIMO IP Address Last 3 Digits ---
fprintf('Enter the last 3 digits of LIMO IP address\n');
fprintf('Example: Enter "101" for 192.168.1.101\n');
LIMO_IP_LAST_3 = input('IP last 3 digits: ', 's');

% --- LIMO Number ---
fprintf('\nEnter LIMO identification number\n');
fprintf('Example: "777" or "807"\n');
LIMO_NUMBER = input('LIMO number: ', 's');

fprintf('\nConfiguration Summary:\n');
fprintf('  Route: %d\n', route_num);
fprintf('  LIMO IP: 192.168.1.%s\n', LIMO_IP_LAST_3);
fprintf('  LIMO Number: %s\n', LIMO_NUMBER);
fprintf('========================================\n\n');

%% ===============================================
%% ROUTE-SPECIFIC CONFIGURATION
%% ===============================================
% Set obstacles, goal, and planned path based on selected route
switch route_num
    case 1
        % Route 1: Two-column obstacle course with right-side path
        fprintf('Loading Route 1 configuration...\n');
        
        % Obstacles: Two vertical columns at x=1.5 and x=3.5
        obs_xy = [1.5, 1.5,    1.5,    1.5,    1.5,    1.5, 3.5, 3.5,    3.5,    3.5,    3.5,    3.5;
                  0.0, 0.5,    1.0,    1.5,    2.0,    2.5, 4.5, 4.0,    3.5,    3.0,    2.5,    2.0];
        
        % Goal position [x; y]
        Goal = [5.0; 4.5];
        
        % Planned path waypoints (flipped from original reverse order)
        rx_temp = [5.0, 4.5, 4.0, 4.0, 4.0, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0];
        ry_temp = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 2.0, 2.5, 3.0, 3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0];
        
        % Flip arrays to get correct forward direction (start -> goal)
        rx = fliplr(rx_temp);
        ry = fliplr(ry_temp);
        
    case 2
        % Route 2: 3x3 grid obstacle pattern
        fprintf('Loading Route 2 configuration...\n');
        
        % Obstacles: 3x3 grid at specific locations
        obs_xy = [1.0, 2.5, 4.0, 0.0, 1.5, 3.0, 1.0, 2.5, 4.0;
                  1.0, 1.0, 1.0, 2.5, 2.5, 2.5, 4.0, 4.0, 4.0];
        
        % Goal position [x; y]
        Goal = [5.0; 4.5];
        
        % Planned path waypoints (as provided in original code)
        rx = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 3.5, 4.0, 4.5, 5.0];
        ry = [0.0, 0.0, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5];
        
    case 3
        % Route 3: Complex S-shaped path with multiple obstacles
        fprintf('Loading Route 3 configuration...\n');
        
        % Obstacles: Complex S-shaped pattern
        obs_xy = [0.0, 0.5, 1.0, 1.0, 1.0, 1.5, 2.0, 2.5, 2.5, 2.5, 2.5, 3.0, 3.5, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 0.0, 0.5, 1.0, 2.5, 2.5;
                  1.0, 1.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 2.5, 3.0, 3.5, 3.5, 3.5, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 3.5, 3.5, 3.5, 0.0, 0.5];
        
        % Goal position [x; y]
        Goal = [0.0; 1.5];
        
        % Planned path waypoints (as provided in original code)
        rx = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0];
        ry = [0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 0.5, 1.0, 1.0, 2.0, 2.5, 3.0, 3.5, 4.0, 4.0, 4.0, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5];
        
    otherwise
        % This should never happen due to validation above
        error('Invalid route number');
end

% Display route configuration
fprintf('  Obstacles: %d points\n', size(obs_xy, 2));
fprintf('  Goal: [%.1f, %.1f]\n', Goal(1), Goal(2));
fprintf('  Path waypoints: %d points\n\n', length(rx));

%% ===============================================
%% CONFIGURATION
%% ===============================================
% LIMO / Network Configuration
CFG.limo_ip_prefix = '192.168.1.';
CFG.limo_port = 12345;
CFG.mqtt_broker = 'mqtt://rasticvm.lan';

% MoCap origin transformation
CFG.MOCAP_ORIGIN_X = -4.5;
CFG.MOCAP_ORIGIN_Y = 2.7;

% Pure Pursuit Controller Parameters
CFG.LOOKAHEAD_DISTANCE = 0.5;   % Lookahead distance [m]
CFG.V_DESIRED = 0.25;           % Desired linear velocity [m/s]
CFG.MIN_TURNING_RADIUS = 0.25;  % Minimum turning radius for Dubins curves [m]

% Loop Recovery Parameters
CFG.LOOP_RECOVERY_DURATION = 1.0;      % Time to execute recovery maneuver [s]
CFG.LOOPS_BEFORE_RECOVERY = 2;         % Number of loops before triggering recovery

% Goal tolerance
CFG.GOAL_POSITION_TOL = 0.10;          % Position tolerance to consider goal reached [m]
CFG.GOAL_HEADING_TOL = deg2rad(25);    % Heading tolerance to consider goal reached [rad]

% Safety limits
CFG.MAX_LINEAR_VEL = 0.3;              % Maximum linear velocity [m/s]
CFG.MAX_ANGULAR_VEL = deg2rad(60);     % Maximum angular velocity [rad/s]
CFG.MAX_TIME = 360;                    % Maximum execution time [s]

% Control loop timing
CFG.CONTROL_RATE = 20;                 % Control loop frequency [Hz]
CFG.dt = 1/CFG.CONTROL_RATE;           % Time step [s]

% Initialize communication objects
tcp = [];
mqttClient = [];

%% ===============================================
%% EKF INITIALIZATION AND CONFIGURATION
%% ===============================================

% --- 1. Define EKF Tuning Parameters (Q and R) ---
% Q: Process Noise Covariance (How much we trust the LIMO's kinematic model)
Q = diag([0.005, 0.005, 0.001]); % [x, y, theta] uncertainty
% R: Measurement Noise Covariance (How much we trust the MoCap sensor)
R = diag([0.0001, 0.0001, 0.0001]); % [x, y, theta] - MoCap is highly accurate.

% Get first raw MoCap measurement (z_0) to set the initial state
fprintf('Initializing EKF with first MoCap reading...\n');
[z_0, ~] = readPosAndRot(mqttClient, CFG, LIMO_NUMBER); 

% Initial State Estimate (x_hat_prev)
x_hat_prev = z_0; 

% Initial Covariance (P_prev): High uncertainty for the first guess
P_prev = eye(3) * 10; 

% Last control command sent (Needed for the prediction step in the first loop)
last_u = [0; 0]; % [v; omega]

% Set the control loop time step (dt)
dt = CFG.dt;

try
    %% ===============================================
    %% VALIDATE AND PREPARE PATH
    %% ===============================================
    fprinitf('Validating planner path...\n');
    
    % Get number of waypoints
    N = length(rx);
    
    % Check that rx and ry have same length
    if length(ry) ~= N
        error('rx and ry must have same length');
    end
    
    fprintf('  Path validated: %d waypoints\n', N);
    
    %% ===============================================
    %% COMPUTE WAYPOINT HEADINGS
    %% ===============================================
    % Compute heading angle between consecutive waypoints
    waypoints = zeros(N,3);  % [x, y, theta] for each waypoint
    
    for i = 1:N-1
        dx = rx(i+1) - rx(i);
        dy = ry(i+1) - ry(i);
        theta = atan2(dy, dx);
        waypoints(i,:) = [rx(i), ry(i), theta];
    end
    
    % Last waypoint uses same heading as second-to-last segment
    waypoints(N,:) = [rx(N), ry(N), waypoints(N-1,3)];
    
    
    %% ===============================================
    %% MQTT CONNECTION
    %% ===============================================
    fprintf('Connecting to MQTT broker: %s\n', CFG.mqtt_broker);
    mqttClient = mqttclient(CFG.mqtt_broker);
    % Subscribe to position and rotation topics for this specific LIMO
    subscribe(mqttClient, sprintf("rb/limo%s/pos", LIMO_NUMBER));
    subscribe(mqttClient, sprintf("rb/limo%s/rot", LIMO_NUMBER));

    % WAIT 10 SECONDS FOR MQTT DATA TO STABILIZE
    fprintf('\nWaiting for MoCap data to stabilize...\n');
    wait_time = 10;  % seconds
    
    for i = 1:wait_time
        fprintf('  %d/%d seconds... ', i, wait_time);
        
        % Try reading data to verify connection (using the EKF helper function)
        % Note: This block uses the simplified helper for initial check.
        [test_pose, valid] = readPosAndRot(mqttClient, CFG, LIMO_NUMBER);
        
        if valid
            fprintf('✓ Data received: (%.2f, %.2f, %.1f°)\n', ...
                    test_pose(1), test_pose(2), rad2deg(test_pose(3)));
        else
            fprintf('⚠ No data yet\n');
        end
        
        pause(1);
    end

    fprintf('Wait complete.\n\n');

    % Get initial pose with multiple attempts
    fprintf('Reading initial robot position...\n');
    valid = false;
    max_attempts = 10;
    curr_pose = x_hat_prev; % Start with the EKF initial state
    
    % Use the EKF state which was initialized above
    start_pose = x_hat_prev; 
    
    % --- Initial pose lock check is now skipped as EKF provides a starting point ---
    fprintf('✓ Initial pose locked via EKF initialization: (%.2f, %.2f, %.1f°)\n', ...
            start_pose(1), start_pose(2), rad2deg(start_pose(3)));
    
    %% ===============================================
    %% LIMO TCP CONNECTION
    %% ===============================================
    % Construct full IP address using prefix and user-provided last 3 digits
    limo_ip = [CFG.limo_ip_prefix LIMO_IP_LAST_3];
    fprintf('Connecting to LIMO TCP: %s:%d\n', limo_ip, CFG.limo_port);
    
    % Create TCP client connection to LIMO
    tcp = tcpclient(limo_ip, CFG.limo_port, 'Timeout', 5);
    
    % Send initial stop command (0 velocity)
    write(tcp, uint8('0.00,0.00')); 
    pause(0.5);
    
    %% ===============================================
    %% GENERATE DUBINS PATH FOR ALL WAYPOINTS
    %% ===============================================
    fprintf('Generating Dubins path along waypoints...\n');
    
    % Initialize empty array to store full Dubins path
    dubinsPathFull = [];
    
    % Create Dubins connection object with minimum turning radius
    dubinsObj = dubinsConnection('MinTurningRadius', CFG.MIN_TURNING_RADIUS);
    
    % Generate smooth Dubins curves between consecutive waypoints
    for i = 1:N-1
        start_wp = waypoints(i,:);
        goal_wp = waypoints(i+1,:);
        [segObj,~] = connect(dubinsObj, start_wp, goal_wp);
        
        if isempty(segObj)
            error('Dubins path between waypoints %d-%d failed', i, i+1);
        end
        
        segPoints = interpolate(segObj{1}, 0:0.05:segObj{1}.Length);
        
        if i>1
            segPoints = segPoints(2:end,:);
        end
        
        dubinsPathFull = [dubinsPathFull; segPoints];
    end
    
    % Extract path coordinates and calculate arc length
    path.x = dubinsPathFull(:,1);  % X coordinates [m]
    path.y = dubinsPathFull(:,2);  % Y coordinates [m]
    
    % Calculate cumulative arc length along path
    path.s = [0; cumsum(sqrt(diff(path.x).^2 + diff(path.y).^2))];
    
    fprintf('Dubins path generated: %d points, length = %.2f m\n', ...
            length(path.x), path.s(end));
    
    %% ===============================================
    %% VISUALIZATION SETUP
    %% ===============================================
    figure('Name','LIMO Path Following','Position',[100 100 1000 800]);
    
    % Plot elements
    plot(obs_xy(1,:), obs_xy(2,:), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    hold on; grid on; axis equal;
    plot(rx, ry, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Waypoints');
    plot(path.x, path.y, 'c-', 'LineWidth', 2, 'DisplayName', 'Dubins Path');
    plot(Goal(1), Goal(2), 'g*', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', 'Goal');
    plot(start_pose(1), start_pose(2), 'ms', 'MarkerSize', 15, ...
         'MarkerFaceColor', 'm', 'DisplayName', 'Start');
    
    % Initialize robot trajectory plot
    trajectory_plot = plot(NaN, NaN, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Actual Path');
    robot_plot = plot(NaN, NaN, 'ko', 'MarkerSize', 12, ...
                     'MarkerFaceColor', 'y', 'DisplayName', 'Robot');
    lookahead_plot = plot(NaN, NaN, 'mo', 'MarkerSize', 10, ...
                         'MarkerFaceColor', 'm', 'DisplayName', 'Lookahead');
    
    % Labels and legend
    xlabel('X [m]'); ylabel('Y [m]');
    title(sprintf('LIMO %s - Pure Pursuit Control (Route %d)', LIMO_NUMBER, route_num));
    legend('Location', 'best');
    
    %% ===============================================
    %% CONTROL LOOP
    %% ===============================================
    fprintf('\nStarting control loop...\n');
    fprintf('Goal: (%.2f, %.2f)\n', Goal(1), Goal(2));
    
    % Initialize control variables
    start_time = tic;                    % Start timer for elapsed time
    trajectory = start_pose(1:2)';       % Store trajectory for plotting [N x 2]
    
    % Loop detection variables
    loop_history = [];
    loop_counter = 0;
    recovery_start_time = [];
    
    % Main control loop - runs until goal is reached or timeout
    while true
        loop_start = tic;  % Time this iteration for rate control
        
        % --- 1. Get RAW MoCap measurement (z_k) ---
        [z_k, valid] = readPosAndRot(mqttClient, CFG, LIMO_NUMBER);
        
        % --- 2. EKF Core Logic (Prediction + Update) ---
        if valid
            % Pass previous state, previous covariance, last control input, new measurement, dt, Q, R
            [x_hat_new, P_new] = kalmanFilter(x_hat_prev, P_prev, last_u, z_k, dt, Q, R);
            
            % Update persistent variables for the next cycle
            x_hat_prev = x_hat_new;
            P_prev = P_new;
            
            % USE FILTERED STATE FOR PURE PURSUIT CONTROL
            curr_x = x_hat_new(1);
            curr_y = x_hat_new(2);
            curr_theta = x_hat_new(3);
        else
            % If MoCap data is invalid, maintain the state from the previous valid loop
            fprintf('⚠ Lost MoCap data - maintaining previous EKF state.\n');
            curr_x = x_hat_prev(1);
            curr_y = x_hat_prev(2);
            curr_theta = x_hat_prev(3);
            % Skip command calculation, use last command
            v_cmd = last_u(1);
            w_cmd = last_u(2);
            % Go directly to sending command and cleanup
            
            % If you choose to STOP the robot on data loss instead, use:
            % v_cmd = 0.0;
            % w_cmd = 0.0;
            
            % Skip the rest of the control logic below and jump to command sending
            cmd_str = sprintf('%.2f,%.2f', v_cmd, w_cmd);
            write(tcp, uint8(cmd_str));
            
            % Update loop time for the next iteration
            elapsed = toc(loop_start);
            if elapsed < CFG.dt
                pause(CFG.dt - elapsed);
            end
            continue; % Skip to next loop iteration
        end
        
        % Calculate elapsed time
        curr_time = toc(start_time);
        
        % --- Check if goal is reached ---
        dist_to_goal = sqrt((curr_x - Goal(1))^2 + (curr_y - Goal(2))^2);
        
        if dist_to_goal < CFG.GOAL_POSITION_TOL
            fprintf('\n✓ Goal reached! Time: %.1f s\n', curr_time);
            write(tcp, uint8('0.00,0.00'));  % Stop robot
            break;
        end
        
        % --- Pure Pursuit: Find lookahead point ---
        [lookahead_x, lookahead_y, lookahead_idx, crosstrack_error] = ...
            findLookaheadPoint(curr_x, curr_y, path, CFG.LOOKAHEAD_DISTANCE);
        
        % --- Loop Detection Logic ---
        loop_history = [loop_history, crosstrack_error];
        if length(loop_history) > 30
            loop_history = loop_history(end-29:end);
        end
        
        if length(loop_history) >= 30
            sign_changes = sum(diff(sign(loop_history)) ~= 0);
            
            if sign_changes >= 20
                loop_counter = loop_counter + 1;
                fprintf('⚠ Loop detected! Count: %d\n', loop_counter);
                
                if loop_counter >= CFG.LOOPS_BEFORE_RECOVERY && isempty(recovery_start_time)
                    fprintf('→ Starting recovery maneuver\n');
                    recovery_start_time = tic;
                    loop_counter = 0;
                    loop_history = [];
                end
            end
        end
        
        % --- Execute Recovery Maneuver if Needed ---
        if ~isempty(recovery_start_time)
            recovery_elapsed = toc(recovery_start_time);
            
            if recovery_elapsed < CFG.LOOP_RECOVERY_DURATION
                v_cmd = -CFG.V_DESIRED * 0.5;
                w_cmd = 0.0;
                fprintf('  Recovery: backing up (%.1f/%.1f s)\n', ...
                       recovery_elapsed, CFG.LOOP_RECOVERY_DURATION);
            else
                fprintf('✓ Recovery complete\n');
                recovery_start_time = [];
            end
        else
            % --- Normal Pure Pursuit Control ---
            dx = lookahead_x - curr_x;
            dy = lookahead_y - curr_y;
            desired_theta = atan2(dy, dx);
            
            theta_error = wrapToPi(desired_theta - curr_theta);
            lookahead_dist = sqrt(dx^2 + dy^2);
            
            % Pure Pursuit angular velocity formula
            w_cmd = (2 * CFG.V_DESIRED * sin(theta_error)) / lookahead_dist;
            
            % Linear velocity: reduce speed when heading error is large
            v_cmd = CFG.V_DESIRED * cos(theta_error);
        end
        
        % --- Apply velocity limits (safety) ---
        v_cmd = max(-CFG.MAX_LINEAR_VEL, min(v_cmd, CFG.MAX_LINEAR_VEL));
        w_cmd = max(-CFG.MAX_ANGULAR_VEL, min(w_cmd, CFG.MAX_ANGULAR_VEL));
        
        % --- Send velocity command to LIMO ---
        cmd_str = sprintf('%.2f,%.2f', v_cmd, w_cmd);
        write(tcp, uint8(cmd_str));

        % --- Save command for EKF prediction in the next loop ---
        last_u = [v_cmd; w_cmd]; 
        
        % --- Update visualization ---
        trajectory = [trajectory; curr_x, curr_y];
        
        set(robot_plot, 'XData', curr_x, 'YData', curr_y);
        set(trajectory_plot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        set(lookahead_plot, 'XData', lookahead_x, 'YData', lookahead_y);
        
        % Update title with status information
        title(sprintf('LIMO %s - Route %d | t=%.1fs | d2goal=%.2fm | CTE=%.3fm | v=%.2f, w=%.1f°/s', ...
                     LIMO_NUMBER, route_num, curr_time, dist_to_goal, crosstrack_error, ...
                     v_cmd, rad2deg(w_cmd)));
        
        drawnow limitrate;
        
        % --- Safety timeout check ---
        if curr_time > CFG.MAX_TIME
            fprintf('✗ Timeout reached\n'); 
            break;
        end
        
        % --- Maintain control loop rate ---
        elapsed = toc(loop_start);
        if elapsed < CFG.dt
            pause(CFG.dt - elapsed);
        end
    end
    
catch ME
    % Error handling
    fprintf('Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('  In: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
end

%% ===============================================
%% CLEANUP
%% ===============================================
% Ensure robot is stopped and connections are closed
if ~isempty(tcp) && isvalid(tcp)
    write(tcp, uint8('0.00,0.00'));
    pause(0.5); 
    clear tcp;
end
if ~isempty(mqttClient)
    clear mqttClient;
end
fprintf('Script complete.\n\n');
end

%% ===============================================
%% HELPER FUNCTION: FIND LOOKAHEAD POINT
%% ===============================================

function [lookahead_x, lookahead_y, lookahead_idx, crosstrack_error] = ...
    findLookaheadPoint(robot_x, robot_y, path, lookahead_distance)
% FINDLOOKAHEADPOINT - Find point on path at lookahead distance ahead

    % --- Find closest point on path to robot ---
    distances = sqrt((path.x - robot_x).^2 + (path.y - robot_y).^2);
    [crosstrack_error, closest_idx] = min(distances);
    
    % --- Find lookahead point ---
    s_closest = path.s(closest_idx);
    s_lookahead = s_closest + lookahead_distance;
    
    if s_lookahead > path.s(end)
        lookahead_idx = length(path.x);
    else
        lookahead_idx = find(path.s >= s_lookahead, 1, 'first');
        
        if isempty(lookahead_idx)
            lookahead_idx = length(path.x);
        end
    end
    
    lookahead_x = path.x(lookahead_idx);
    lookahead_y = path.y(lookahead_idx);
end

%% ===============================================
%% HELPER FUNCTION: READ POSITION AND ROTATION
%% ===============================================

function [z_k, valid] = readPosAndRot(mqttClient, CFG, LIMO_NUMBER)
% READPOSANDROT - Reads, parses, and transforms the latest MoCap data from MQTT.
% Returns z_k: [x; y; theta] in MAP coordinates.

    valid = false;
    z_k = [NaN; NaN; NaN];
    
    % --- Topics ---
    posTopic = sprintf('rb/limo%s/pos', LIMO_NUMBER);
    rotTopic = sprintf('rb/limo%s/rot', LIMO_NUMBER);
    
    % --- Read Data ---
    posTable = read(mqttClient, Topic=posTopic);
    rotTable = read(mqttClient, Topic=rotTopic);

    if ~isempty(posTable) && ~isempty(rotTable)
        try
            % 1. Parse Position Data
            posData = posTable.Data{end};
            posData = erase(posData, ['[', ']']);
            mocapPos = str2double(split(posData, ','))'; % [X, Z, Y]
            
            % 2. Parse Rotation Data
            rotData = rotTable.Data{end};
            rotData = erase(rotData, ['[', ']']);
            mocapRot = str2double(split(rotData, ','))'; % [Roll, Pitch, Yaw]

            % 3. Apply MoCap to Map Transformation (using CFG variables)
            x_mocap = mocapPos(1);  % X from MoCap
            y_mocap = mocapPos(3);  % Y from MoCap (third element)
            theta_mocap = mocapRot(3); % Yaw (heading) from MoCap
            
            % Transformation (subtract MoCap origin to get Map coordinates)
            x_map = x_mocap - CFG.MOCAP_ORIGIN_X; 
            y_map = y_mocap - CFG.MOCAP_ORIGIN_Y;
            
            % Final measurement vector [x; y; theta]
            z_k = [x_map; y_map; theta_mocap];
            valid = true;
            
        catch
            % Parsing or conversion failed
        end
    end
end
