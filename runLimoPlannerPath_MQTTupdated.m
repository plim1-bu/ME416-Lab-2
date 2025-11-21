function runLimoPlannerPath_MQTTupdated()
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
        % Original was backwards, so we reverse it here
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
% (LIMO_NUMBER and LIMO_IP_LAST_3 are now set by user input above)
CFG.limo_ip_prefix = '192.168.1.';
CFG.limo_port = 12345;
CFG.mqtt_broker = 'mqtt://rasticvm.lan';

% MoCap origin transformation
% These values transform MoCap coordinates to map coordinates
CFG.MOCAP_ORIGIN_X = -4.5;
CFG.MOCAP_ORIGIN_Y = 2.7;

% Pure Pursuit Controller Parameters
CFG.LOOKAHEAD_DISTANCE = 0.5;   % Lookahead distance [m]
CFG.V_DESIRED = 0.25;           % Desired linear velocity [m/s]
CFG.MIN_TURNING_RADIUS = 0.25;  % Minimum turning radius for Dubins curves [m]

% Loop Recovery Parameters
% These help detect and recover from circular motion
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

try
    %% ===============================================
    %% VALIDATE AND PREPARE PATH
    %% ===============================================
    fprintf('Validating planner path...\n');
    
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
    % Heading is the direction the robot should face to move from one waypoint to the next
    waypoints = zeros(N,3);  % [x, y, theta] for each waypoint
    
    for i = 1:N-1
        % Calculate displacement vector between waypoints
        dx = rx(i+1) - rx(i);
        dy = ry(i+1) - ry(i);
        
        % Calculate heading angle using atan2 (handles all quadrants correctly)
        theta = atan2(dy, dx);
        
        % Store waypoint with position and heading
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
subscribe(mqttClient, sprintf("rb/limo%s", LIMO_NUMBER));

% WAIT 10 SECONDS FOR MQTT DATA TO STABILIZE
% This ensures motion capture system is publishing valid data
fprintf('\nWaiting for MoCap data to stabilize...\n');
wait_time = 10;  % seconds
for i = 1:wait_time
    fprintf('  %d/%d seconds... ', i, wait_time);
    
    % Try reading data to verify connection
    [test_pose, valid] = getRobotPose_MQTT(mqttClient, LIMO_NUMBER, CFG, [], 0);
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
for attempt = 1:max_attempts
    [curr_pose, valid] = getRobotPose_MQTT(mqttClient, LIMO_NUMBER, CFG, [], 0);
    if valid
        fprintf('✓ Initial pose locked: (%.2f, %.2f, %.1f°)\n', ...
                curr_pose(1), curr_pose(2), rad2deg(curr_pose(3)));
        break;
    end
    fprintf('  Attempt %d/%d: No valid data...\n', attempt, max_attempts);
    pause(0.5);
end

% Check if initial pose was successfully obtained
if ~valid
    error(['Cannot read initial MoCap data after %d seconds!\n' ...
           'Check:\n' ...
           '  1. Motion capture system is running\n' ...
           '  2. LIMO%s rigid body is tracked\n' ...
           '  3. Data is being published to MQTT\n' ...
           '  4. Robot is NOT in MoCap dead zone near (0,0)'], ...
           wait_time + max_attempts*0.5, LIMO_NUMBER);
end
start_pose = curr_pose;

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
        % Get start and goal waypoints for this segment
        start_wp = waypoints(i,:);
        goal_wp = waypoints(i+1,:);
        
        % Generate Dubins path between waypoints
        [segObj,~] = connect(dubinsObj, start_wp, goal_wp);
        
        % Check if path generation was successful
        if isempty(segObj)
            error('Dubins path between waypoints %d-%d failed', i, i+1);
        end
        
        % Interpolate points along Dubins curve (every 0.05m)
        segPoints = interpolate(segObj{1}, 0:0.05:segObj{1}.Length);
        
        if i>1
            % Remove duplicate first point (it's the same as last point of previous segment)
            segPoints = segPoints(2:end,:);
        end
        
        % Append segment points to full path
        dubinsPathFull = [dubinsPathFull; segPoints];
    end
    
    % Extract path coordinates and calculate arc length
    path.x = dubinsPathFull(:,1);  % X coordinates [m]
    path.y = dubinsPathFull(:,2);  % Y coordinates [m]
    
    % Calculate cumulative arc length along path
    % Arc length s(i) = sum of distances from start to point i
    path.s = [0; cumsum(sqrt(diff(path.x).^2 + diff(path.y).^2))];
    
    fprintf('Dubins path generated: %d points, length = %.2f m\n', ...
            length(path.x), path.s(end));
    
    %% ===============================================
    %% VISUALIZATION SETUP
    %% ===============================================
    figure('Name','LIMO Path Following','Position',[100 100 1000 800]);
    
    % Plot obstacles as red circles
    plot(obs_xy(1,:), obs_xy(2,:), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    hold on; grid on; axis equal;
    
    % Plot planned waypoints as blue line
    plot(rx, ry, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Waypoints');
    
    % Plot Dubins path as cyan line
    plot(path.x, path.y, 'c-', 'LineWidth', 2, 'DisplayName', 'Dubins Path');
    
    % Plot goal position as green star
    plot(Goal(1), Goal(2), 'g*', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', 'Goal');
    
    % Plot start position as magenta square
    plot(start_pose(1), start_pose(2), 'ms', 'MarkerSize', 15, ...
         'MarkerFaceColor', 'm', 'DisplayName', 'Start');
    
    % Initialize robot trajectory plot (will be updated in real-time)
    trajectory_plot = plot(NaN, NaN, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Actual Path');
    
    % Initialize robot position marker
    robot_plot = plot(NaN, NaN, 'ko', 'MarkerSize', 12, ...
                     'MarkerFaceColor', 'y', 'DisplayName', 'Robot');
    
    % Initialize lookahead point marker
    lookahead_plot = plot(NaN, NaN, 'mo', 'MarkerSize', 10, ...
                         'MarkerFaceColor', 'm', 'DisplayName', 'Lookahead');
    
    % Add labels and legend
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
    curr_pose = start_pose;              % Current robot pose [x, y, theta]
    trajectory = start_pose(1:2);        % Store trajectory for plotting (1x2 row vector)
    
    % Loop detection variables (to detect if robot is going in circles)
    loop_history = [];                   % History of crosstrack errors
    loop_counter = 0;                    % Count of detected loops
    recovery_start_time = [];            % Timer for recovery maneuver
    
    % Main control loop - runs until goal is reached or timeout
    while true
        loop_start = tic;  % Time this iteration for rate control
        
        % --- Get current robot state from MoCap ---
        prev_pose = curr_pose;
        [curr_pose, valid] = getRobotPose_MQTT(mqttClient, LIMO_NUMBER, CFG, prev_pose, CFG.dt);
        
        if ~valid
            fprintf('⚠ Lost MoCap data - maintaining previous pose\n');
            continue;  % Skip this iteration and try again
        end
        
        % Extract current position and heading
        curr_x = curr_pose(1);
        curr_y = curr_pose(2);
        curr_theta = curr_pose(3);
        
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
        % Keep track of recent crosstrack errors to detect circular motion
        loop_history = [loop_history, crosstrack_error];
        if length(loop_history) > 30
            loop_history = loop_history(end-29:end);  % Keep last 30 samples
        end
        
        % Detect loop: crosstrack error oscillates regularly (robot going in circles)
        if length(loop_history) >= 30
            % Check if error alternates sign (crosses path repeatedly)
            sign_changes = sum(diff(sign(loop_history)) ~= 0);
            
            if sign_changes >= 20  % More than 20 sign changes in 30 samples
                loop_counter = loop_counter + 1;
                fprintf('⚠ Loop detected! Count: %d\n', loop_counter);
                
                % Trigger recovery maneuver after detecting multiple loops
                if loop_counter >= CFG.LOOPS_BEFORE_RECOVERY && isempty(recovery_start_time)
                    fprintf('→ Starting recovery maneuver\n');
                    recovery_start_time = tic;  % Start recovery timer
                    loop_counter = 0;           % Reset loop counter
                    loop_history = [];          % Clear history
                end
            end
        end
        
        % --- Execute Recovery Maneuver if Needed ---
        if ~isempty(recovery_start_time)
            recovery_elapsed = toc(recovery_start_time);
            
            if recovery_elapsed < CFG.LOOP_RECOVERY_DURATION
                % During recovery: stop and rotate to face path
                v_cmd = 0.0;                    % Stop (no backward motion!)
                w_cmd = deg2rad(30);            % Rotate to reorient
                fprintf('  Recovery: rotating to reorient (%.1f/%.1f s)\n', ...
                       recovery_elapsed, CFG.LOOP_RECOVERY_DURATION);
            else
                % Recovery complete
                fprintf('✓ Recovery complete\n');
                recovery_start_time = [];  % Clear recovery timer
            end
        else
            % --- Normal Pure Pursuit Control ---
            % Calculate desired heading to lookahead point
            dx = lookahead_x - curr_x;
            dy = lookahead_y - curr_y;
            desired_theta = atan2(dy, dx);
            
            % Calculate heading error (wrap to [-pi, pi])
            theta_error = wrapToPi(desired_theta - curr_theta);
            
            % Calculate distance to lookahead point
            lookahead_dist = sqrt(dx^2 + dy^2);
            
            % Pure Pursuit angular velocity formula
            % w = (2 * v * sin(theta_error)) / lookahead_distance
            w_cmd = (2 * CFG.V_DESIRED * sin(theta_error)) / lookahead_dist;
            
            % Linear velocity: reduce speed when heading error is large
            % But ALWAYS maintain minimum forward velocity (never stop unless at goal)
            if abs(theta_error) < deg2rad(90)
                % Heading error < 90°: scale velocity by cos(error)
                v_cmd = CFG.V_DESIRED * cos(theta_error);
            else
                % Heading error > 90°: use minimum forward speed and let rotation fix heading
                v_cmd = CFG.V_DESIRED * 0.3;  % 30% speed when heading is very wrong
            end
        end
        
        % --- Apply velocity limits (safety) ---
        % CRITICAL: Clamp velocity to NEVER be negative (forward only!)
        v_cmd = max(0, min(v_cmd, CFG.MAX_LINEAR_VEL));  % Always >= 0
        w_cmd = max(-CFG.MAX_ANGULAR_VEL, min(w_cmd, CFG.MAX_ANGULAR_VEL));
        
        % Debug: Show if velocity was clamped to 0
        if v_cmd == 0
            fprintf('  WARNING: Velocity clamped to 0! Check heading error.\n');
        end
        
        % --- Send velocity command to LIMO ---
        cmd_str = sprintf('%.2f,%.2f', v_cmd, w_cmd);
        write(tcp, uint8(cmd_str));
        
        % --- Update visualization ---
        trajectory = [trajectory; curr_x, curr_y];  % Append current position
        
        set(robot_plot, 'XData', curr_x, 'YData', curr_y);
        set(trajectory_plot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        set(lookahead_plot, 'XData', lookahead_x, 'YData', lookahead_y);
        
        % Update title with status information
        title(sprintf('LIMO %s - Route %d | t=%.1fs | d2goal=%.2fm | CTE=%.3fm | v=%.2f, w=%.1f°/s', ...
                     LIMO_NUMBER, route_num, curr_time, dist_to_goal, crosstrack_error, ...
                     v_cmd, rad2deg(w_cmd)));
        
        drawnow limitrate;  % Update figure (rate-limited for performance)
        
        % --- Safety timeout check ---
        if curr_time > CFG.MAX_TIME
            fprintf('✗ Timeout reached\n'); 
            break;
        end
        
        % --- Maintain control loop rate ---
        elapsed = toc(loop_start);
        if elapsed < CFG.dt
            pause(CFG.dt - elapsed);  % Wait to maintain desired rate
        end
    end
    
catch ME
    % Error handling: display error message and location
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
    write(tcp, uint8('0.00,0.00'));  % Send stop command
    pause(0.5); 
    clear tcp;  % Close TCP connection
end
if ~isempty(mqttClient)
    clear mqttClient;  % Close MQTT connection
end

fprintf('Script complete.\n\n');

end

%% ===============================================
%% HELPER FUNCTION: GET ROBOT POSE FROM MQTT
%% ===============================================

function [pose, valid] = getRobotPose_MQTT(mqttClient, limoNum, CFG, prev_pose, dt)
% GETROBOTPOSE_MQTT - Read robot pose from MQTT MoCap
%
% HEADING CALCULATION:
% Instead of using rotation data (which can be noisy/misaligned),
% we calculate heading from the DIRECTION OF MOTION (velocity vector).
% heading = atan2(dy/dt, dx/dt)
%
% INPUTS:
%   mqttClient - MQTT client object
%   limoNum    - LIMO number as string (e.g., '809')
%   CFG        - Configuration structure with:
%                  .MOCAP_ORIGIN_X - X offset for coordinate transform [m]
%                  .MOCAP_ORIGIN_Y - Y offset for coordinate transform [m]
%   prev_pose  - Previous pose [x, y, theta] for velocity calculation
%   dt         - Time step since last reading [s]
%
% OUTPUTS:
%   pose  - [x, y, theta] where theta is from velocity direction [rad]
%   valid - Boolean indicating if data was successfully read

    % Initialize output values
    pose = [0, 0, 0];
    valid = false;
    
    try
        % --- Read Data from MQTT (JSON Format) ---
        % Use peek() to read latest message without removing from queue
        mqttMsg = peek(mqttClient);
        
        % Check if data was received
        if isempty(mqttMsg)
            return;  % No data available, exit with valid = false
        end
        
        % Verify message is from correct robot topic
        expected_topic = sprintf('rb/limo%s', limoNum);
        if ~strcmp(char(mqttMsg.Topic), expected_topic)
            return;  % Wrong topic, exit with valid = false
        end
        
        % Parse JSON message
        jsonString = char(mqttMsg.Data);
        jsonData = jsondecode(jsonString);
        
        % Validate that required fields exist
        if ~isfield(jsonData, 'pos') || ~isfield(jsonData, 'rot')
            return;  % Invalid data, exit with valid = false
        end
        
        % --- Transform MoCap Coordinates to Map Coordinates ---
        % MoCap format: [X, Z, Y] where Y is vertical (up)
        % We need: [x, y] for 2D navigation
        mocap_x = jsonData.pos(1);  % X coordinate from MoCap
        mocap_y = jsonData.pos(3);  % Y coordinate is 3rd element (not 2nd!)
        
        % Apply coordinate transformation
        % Subtract origin offset to align MoCap frame with map frame
        x = mocap_x - CFG.MOCAP_ORIGIN_X;
        
        % Negate Y to match coordinate system convention
        % (MoCap Y axis may be flipped relative to map Y axis)
        y = -(mocap_y - CFG.MOCAP_ORIGIN_Y);
        
        % --- Calculate Heading from Velocity Direction ---
        if ~isempty(prev_pose) && dt > 0
            % Calculate displacement since last reading
            dx = x - prev_pose(1);  % Change in X position [m]
            dy = y - prev_pose(2);  % Change in Y position [m]
            
            % Calculate robot's speed
            speed = sqrt(dx^2 + dy^2) / dt;  % [m/s]
            
            % Only update heading if robot is moving significantly
            % This prevents noise from affecting heading when stationary
            speed_threshold = 0.05;  % Minimum speed to update heading [m/s]
            
            if speed > speed_threshold
                % Heading = direction of motion (velocity vector angle)
                theta = atan2(dy, dx);  % [rad]
            else
                % Robot is stationary - maintain previous heading
                theta = prev_pose(3);
            end
        else
            % First reading - no previous pose available
            % Read initial heading from rotation data (JSON format)
            if length(jsonData.rot) >= 3
                % Use yaw angle from rotation data as initial heading
                % Negate to match Y-axis flip in coordinate transform
                theta = -jsonData.rot(3);  % [rad]
            else
                theta = 0;  % Default: facing East (0 radians)
            end
        end
        
        % Package pose as [x, y, theta]
        pose = [x, y, theta];
        valid = true;  % Successfully read and processed data
        
    catch ME
        % Error occurred during MQTT read or data processing
        % Silent failure - valid remains false
        % (You can add warning() here if you want to see errors)
    end
end

%% ===============================================
%% HELPER FUNCTION: FIND LOOKAHEAD POINT
%% ===============================================

function [lookahead_x, lookahead_y, lookahead_idx, crosstrack_error] = ...
    findLookaheadPoint(robot_x, robot_y, path, lookahead_distance)
% FINDLOOKAHEADPOINT - Find point on path at lookahead distance ahead
%
% This function implements the Pure Pursuit algorithm's lookahead mechanism.
% It finds the closest point on the path to the robot, then finds a point
% that is 'lookahead_distance' ahead along the path from that closest point.
%
% INPUTS:
%   robot_x, robot_y   - Current robot position [m]
%   path               - Structure with fields:
%                          .x - X coordinates of path points [m]
%                          .y - Y coordinates of path points [m]
%                          .s - Arc length along path at each point [m]
%   lookahead_distance - How far ahead to look on path [m]
%
% OUTPUTS:
%   lookahead_x, lookahead_y - Coordinates of lookahead point [m]
%   lookahead_idx            - Index of lookahead point in path arrays
%   crosstrack_error         - Perpendicular distance from robot to path [m]

    % --- Find closest point on path to robot ---
    % Calculate Euclidean distance from robot to every path point
    distances = sqrt((path.x - robot_x).^2 + (path.y - robot_y).^2);
    
    % Find minimum distance and its index
    [crosstrack_error, closest_idx] = min(distances);
    
    % --- Find lookahead point ---
    % Get arc length at closest point
    s_closest = path.s(closest_idx);
    
    % Desired arc length is lookahead distance ahead
    s_lookahead = s_closest + lookahead_distance;
    
    % Find path point at or beyond the lookahead arc length
    if s_lookahead > path.s(end)
        % Lookahead goes beyond end of path - use goal point
        lookahead_idx = length(path.x);
    else
        % Find first point where arc length >= s_lookahead
        lookahead_idx = find(path.s >= s_lookahead, 1, 'first');
        
        % Handle edge case where find returns empty
        if isempty(lookahead_idx)
            lookahead_idx = length(path.x);
        end
    end
    
    % Extract lookahead point coordinates
    lookahead_x = path.x(lookahead_idx);
    lookahead_y = path.y(lookahead_idx);
end