function runLimoDubins_PurePursuit_MQTT()
% RUNLIMODUBINS_PUREPURSUIT_MQTT - Dubins path planning with Pure Pursuit tracking
%
% WHAT THIS DOES:
% 1. Reads robot's ACTUAL starting position from MQTT MoCap
% 2. Plans optimal Dubins path to user-specified goal (using MATLAB's dubinsConnection)
% 3. Uses Pure Pursuit controller to FOLLOW the Dubins path
% 4. Self-corrects using MoCap feedback if robot drifts off path
%
% KEY FEATURE - VELOCITY-BASED HEADING:
% Instead of using potentially misaligned MoCap rotation data, the robot's
% heading is calculated from its DIRECTION OF MOTION (velocity vector).
% This ensures the heading always matches where the robot is actually moving.
%   heading = atan2(dy/dt, dx/dt)
%   where dy/dt and dx/dt are the velocity components
%
% KEY FEATURE - LOOP RECOVERY:
% If the robot goes in circles near the goal (e.g., overshoots and loops),
% the system detects this and enters RECOVERY MODE:
%   - Detects when robot completes full 360° loops
%   - After 2 loops, switches to driving straight for 1 second
%   - This breaks the circular pattern
%   - Then resumes normal Pure Pursuit control
% You can adjust: CFG.LOOPS_BEFORE_RECOVERY and CFG.LOOP_RECOVERY_DURATION

clearvars;
close all;

% Check for Navigation Toolbox (required for dubinsConnection)
if ~license('test', 'map_toolbox')
    error(['Navigation Toolbox is required for Dubins path planning.\n' ...
           'Install it via: Home > Add-Ons > Get Add-Ons > Search "Navigation Toolbox"']);
end

%% ===============================================
%% CONFIGURATION
%% ===============================================

% --- LIMO Configuration (same as your runLimoFeedback_MQTT) ---
LIMO_NUMBER = '777';      % Your LIMO number
LIMO_IP_LAST_3 = '101';   % Last 3 digits of LIMO IP address

% --- Network Configuration ---
CFG.limo_ip_prefix = '192.168.1.';           % LIMO IP prefix
CFG.limo_port = 12345;                       % LIMO TCP port (NOT 8888!)
CFG.mqtt_broker = 'mqtt://rasticvm.lan';     % MQTT broker address

% --- MoCap Coordinate Transformation (same as Lab2) ---
CFG.MOCAP_ORIGIN_X = -4.5;  % MoCap X for map origin (0,0)
CFG.MOCAP_ORIGIN_Y = 2.7;   % MoCap Y for map origin (0,0)

% --- Pure Pursuit Controller Parameters ---
CFG.LOOKAHEAD_DISTANCE = 0.5;  % How far ahead to look on path [m]
                                % Larger = smoother, cuts corners
                                % Smaller = tight tracking, oscillates
                                
CFG.V_DESIRED = 0.25;           % Desired forward speed [m/s]
CFG.MIN_TURNING_RADIUS = 0.3;   % Minimum turning radius [m] (LIMO constraint)

% --- Loop Recovery Parameters ---
% If robot goes in circles near goal, it will drive straight to break the loop
CFG.LOOP_RECOVERY_DURATION = 1.0;  % How long to drive straight [s]
CFG.LOOPS_BEFORE_RECOVERY = 2;     % Number of loops before recovery kicks in

% --- Goal Tolerance ---
CFG.GOAL_POSITION_TOL = 0.10;      % Within 10 cm = arrived [m]
CFG.GOAL_HEADING_TOL = deg2rad(10); % Within 10° of goal heading [rad]

% --- Safety Limits ---
CFG.MAX_LINEAR_VEL = 0.3;          % Maximum forward speed [m/s]
CFG.MAX_ANGULAR_VEL = deg2rad(60); % Maximum turn rate [rad/s] (60°/s)
CFG.MAX_TIME = 360;                % Maximum execution time [s]

% --- Control Loop Timing ---
CFG.CONTROL_RATE = 20;             % Control frequency [Hz]
CFG.dt = 1/CFG.CONTROL_RATE;       % Control period [s] (0.05s = 50ms)

tcp = [];         % TCP client for LIMO
mqttClient = [];  % MQTT client for MoCap

try
    %% ===============================================
    %% USER INPUT
    %% ===============================================
    
    fprintf('\n========================================\n');
    fprintf('  LIMO DUBINS PATH TRACKER (MQTT)\n');
    fprintf('========================================\n\n');
    
    fprintf('--- Configuration ---\n');
    fprintf('LIMO Number:  %s (from script)\n', LIMO_NUMBER);
    fprintf('LIMO IP:      192.168.1.%s (from script)\n', LIMO_IP_LAST_3);
    fprintf('\n');
    
    % Build LIMO IP
    limoNum = LIMO_NUMBER;
    limo_ip = ['192.168.1.', LIMO_IP_LAST_3];
    
    % --- Get Goal from User ---
    fprintf('Enter GOAL pose (in workspace/map coordinates):\n');
    fprintf('  Note: Origin (0,0) is at MoCap coords (%.1f, %.1f)\n', ...
            CFG.MOCAP_ORIGIN_X, CFG.MOCAP_ORIGIN_Y);
    
    goal_x = input('  Goal X position [m]: ');
    goal_y = input('  Goal Y position [m]: ');
    goal_heading_deg = input('  Goal heading [degrees] (0°=East, 90°=North): ');
    goal_theta = deg2rad(goal_heading_deg);
    
    % Store goal
    goal.x = goal_x;
    goal.y = goal_y;
    goal.theta = goal_theta;
    
    fprintf('\n--- Summary ---\n');
    fprintf('LIMO:         %s @ %s:%d\n', limoNum, limo_ip, CFG.limo_port);
    fprintf('MQTT Broker:  %s\n', CFG.mqtt_broker);
    fprintf('Goal:         (%.2f m, %.2f m, %.1f°)\n', ...
            goal.x, goal.y, goal_heading_deg);
    fprintf('\n');
    
    % Confirm
    confirm = input('Proceed? (y/n): ', 's');
    if ~strcmpi(confirm, 'y')
        fprintf('\nCancelled by user.\n\n');
        return;
    end
    
    %% ===============================================
    %% MQTT CONNECTION
    %% ===============================================
    
    fprintf('\n--- MQTT Connection ---\n');
    fprintf('Connecting to: %s\n', CFG.mqtt_broker);
    
    % Create MQTT client
    mqttClient = mqttclient(CFG.mqtt_broker);
    
    % Subscribe to MoCap topics
    subscribe(mqttClient, sprintf("rb/limo%s/pos", limoNum));
    subscribe(mqttClient, sprintf("rb/limo%s/rot", limoNum));
    
    fprintf('  ✓ Connected to MQTT\n');
    fprintf('  ✓ Subscribed to rb/limo%s/pos\n', limoNum);
    fprintf('  ✓ Subscribed to rb/limo%s/rot\n', limoNum);
    
    % Wait for initial data
    fprintf('  Waiting for MoCap data...\n');
    pause(2.0);
    
    % Verify data (try up to 5 times)
    fprintf('  Checking data availability...\n');
    valid = false;
    for attempt = 1:5
        [curr_pose, valid] = getRobotPose_MQTT(mqttClient, limoNum, CFG, [], 0);
        if valid
            break;
        end
        fprintf('  Attempt %d/5: No data yet...\n', attempt);
        pause(1.0);
    end
    
    if ~valid
        error(['Cannot read MoCap data!\n' ...
               'Check:\n' ...
               '  1. Motion capture system is running\n' ...
               '  2. LIMO%s rigid body is tracked\n' ...
               '  3. Data is being published to MQTT\n' ...
               '  4. Your Lab2 script works'], limoNum);
    end
    
    % Store starting pose
    start.x = curr_pose(1);
    start.y = curr_pose(2);
    start.theta = curr_pose(3);
    
    fprintf('  ✓ Initial pose: (%.2f, %.2f, %.1f°)\n\n', ...
            start.x, start.y, rad2deg(start.theta));
    
    %% ===============================================
    %% LIMO CONNECTION
    %% ===============================================
    
    fprintf('--- LIMO Connection ---\n');
    fprintf('Connecting to: %s:%d\n', limo_ip, CFG.limo_port);
    
    % Create TCP client
    tcp = tcpclient(limo_ip, CFG.limo_port, 'Timeout', 5);
    fprintf('  ✓ Connected to LIMO\n\n');
    
    % Send initial stop command
    write(tcp, uint8('0.00,0.00'));  % Format: 'v,w' where v=linear, w=angular
    pause(0.5);
    
    %% ===============================================
    %% DUBINS PATH PLANNING (MATLAB Built-in)
    %% ===============================================
    
    fprintf('--- Dubins Path Planning ---\n');
    
    % Create Dubins path connection object
    % This object computes shortest Dubins paths with given turning radius
    dubinsPathObj = dubinsConnection('MinTurningRadius', CFG.MIN_TURNING_RADIUS);
    
    % Define start and goal poses as [x, y, theta] vectors
    startPose = [start.x, start.y, start.theta];
    goalPose = [goal.x, goal.y, goal.theta];
    
    fprintf('  Start: (%.2f, %.2f, %.1f°)\n', ...
            startPose(1), startPose(2), rad2deg(startPose(3)));
    fprintf('  Goal:  (%.2f, %.2f, %.1f°)\n', ...
            goalPose(1), goalPose(2), rad2deg(goalPose(3)));
    
    % Compute Dubins path from start to goal
    % Returns: pathSegObj - cell array of path segments
    %          pathCosts - costs (lengths) of each segment type
    [pathSegObj, pathCosts] = connect(dubinsPathObj, startPose, goalPose);
    
    % Check if path was found
    if isempty(pathSegObj)
        error('Dubins path planning failed! Check start/goal poses and turning radius.');
    end
    
    % Get path type by checking the segment motions
    % pathSegObj is a cell array containing the path segments
    % Each segment has a MotionTypes property showing the maneuvers
    try
        motionTypes = pathSegObj{1}.MotionTypes;
        % Convert motion types to string (e.g., {'L', 'S', 'R'} -> 'LSR')
        path_type = strjoin(motionTypes, '');
    catch
        % If we can't get motion types, just label it as 'Dubins'
        path_type = 'Dubins';
    end
    
    % Interpolate the path to get waypoints
    % Use fine resolution for smooth following (every 0.05 m)
    path_resolution = 0.05;  % Distance between waypoints [m]
    totalLength = pathSegObj{1}.Length;  % Total path length [m]
    
    % Generate interpolation distances along path
    interpDistances = 0:path_resolution:totalLength;
    
    % Interpolate path to get [x, y, theta] at each distance
    pathPoints = interpolate(pathSegObj{1}, interpDistances);
    
    % Extract x, y, theta arrays
    path_x = pathPoints(:, 1);
    path_y = pathPoints(:, 2);
    path_theta = pathPoints(:, 3);
    
    fprintf('  ✓ Path type: %s\n', path_type);
    fprintf('  ✓ Path length: %.2f m\n', totalLength);
    fprintf('  ✓ Waypoints: %d\n\n', length(path_x));
    
    % Store path structure
    path.x = path_x;
    path.y = path_y;
    path.theta = path_theta;
    % Calculate cumulative arc length for lookahead calculation
    path.s = cumsum([0; sqrt(diff(path_x).^2 + diff(path_y).^2)]);
    
    %% ===============================================
    %% VISUALIZATION SETUP
    %% ===============================================
    
    fig = figure('Name', 'LIMO Dubins Path Tracker', 'Position', [100 100 1200 800]);
    
    % --- Plot Planned Dubins Path ---
    plot(path.x, path.y, 'b--', 'LineWidth', 2.5, 'DisplayName', 'Planned Dubins Path');
    hold on; grid on; axis equal;
    
    % --- Plot Start ---
    plot(start.x, start.y, 'go', 'MarkerSize', 15, 'LineWidth', 2, ...
         'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    quiver(start.x, start.y, 0.3*cos(start.theta), 0.3*sin(start.theta), ...
           0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);
    
    % --- Plot Goal ---
    plot(goal.x, goal.y, 'ro', 'MarkerSize', 15, 'LineWidth', 2, ...
         'MarkerFaceColor', 'r', 'DisplayName', 'Goal');
    quiver(goal.x, goal.y, 0.3*cos(goal.theta), 0.3*sin(goal.theta), ...
           0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);
    
    % --- Initialize Real-Time Elements ---
    h_robot = plot(start.x, start.y, 'ko', 'MarkerSize', 10, ...
                   'MarkerFaceColor', 'blue', 'DisplayName', 'Robot');
    h_trajectory = plot(start.x, start.y, 'r-', 'LineWidth', 2, ...
                        'DisplayName', 'Actual Trajectory');
    h_lookahead = plot(start.x, start.y, 'cs', 'MarkerSize', 10, ...
                       'MarkerFaceColor', 'c', 'DisplayName', 'Lookahead');
    h_heading = quiver(start.x, start.y, 0, 0, 0, 'm', 'LineWidth', 2, ...
                       'MaxHeadSize', 0.8);
    
    % Labels and legend
    xlabel('X Position [m]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y Position [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('LIMO Dubins Path Tracking with Pure Pursuit', 'FontSize', 14);
    legend('Location', 'best');
    
    % Status text box
    h_text = annotation('textbox', [0.02, 0.85, 0.25, 0.12], ...
                        'String', 'Starting...', ...
                        'FitBoxToText', 'off', ...
                        'BackgroundColor', 'white', ...
                        'EdgeColor', 'black', ...
                        'FontSize', 9, ...
                        'FontName', 'FixedWidth');
    
    %% ===============================================
    %% MAIN CONTROL LOOP
    %% ===============================================
    
    fprintf('--- Path Tracking (Pure Pursuit) ---\n');
    fprintf('Press Ctrl+C to stop\n\n');
    
    % Initialize data logging
    log.time = [];
    log.x = [];
    log.y = [];
    log.theta = [];
    log.v_cmd = [];
    log.omega_cmd = [];
    log.crosstrack_error = [];
    log.heading_error = [];
    
    % Control loop variables
    start_time = tic;
    loop_count = 0;
    goal_reached = false;
    
    traj_x = start.x;  % Trajectory history
    traj_y = start.y;
    
    % Initialize previous pose for velocity-based heading calculation
    prev_pose = [start.x, start.y, start.theta];
    prev_time = 0;
    
    % --- Loop Detection Variables ---
    % Track cumulative heading change to detect circular motion
    cumulative_heading_change = 0;  % Total heading change [rad]
    loop_counter = 0;                % Number of complete loops detected
    recovery_mode = false;           % Flag for straight-line recovery
    recovery_start_time = 0;         % When recovery mode started
    
    % --- Main Loop ---
    while ~goal_reached
        loop_start = tic;
        loop_count = loop_count + 1;
        
        % Current time
        curr_time = toc(start_time);
        dt = curr_time - prev_time;
        
        % --- Get Current Pose from MoCap ---
        % Heading is calculated from velocity direction (direction of motion)
        [curr_pose, valid] = getRobotPose_MQTT(mqttClient, limoNum, CFG, prev_pose, dt);
        
        if ~valid
            warning('Lost MoCap tracking! Skipping iteration...');
            pause(CFG.dt);
            continue;
        end
        
        % Extract current state
        x = curr_pose(1);       % Current X [m]
        y = curr_pose(2);       % Current Y [m]
        theta = curr_pose(3);   % Current heading [rad] - FROM VELOCITY DIRECTION!
        
        % --- Loop Detection Logic ---
        % Calculate heading change since last iteration
        if loop_count > 1
            heading_change = wrapToPi(theta - prev_pose(3));
            cumulative_heading_change = cumulative_heading_change + abs(heading_change);
            
            % Check if completed a full loop (360 degrees = 2*pi radians)
            if cumulative_heading_change >= 2*pi
                loop_counter = loop_counter + 1;
                cumulative_heading_change = 0;  % Reset for next loop detection
                fprintf('  ! Loop detected (#%d)\n', loop_counter);
                
                % If we've looped enough times, enter recovery mode
                if loop_counter >= CFG.LOOPS_BEFORE_RECOVERY
                    recovery_mode = true;
                    recovery_start_time = curr_time;
                    loop_counter = 0;  % Reset loop counter
                    fprintf('  >> RECOVERY MODE: Driving straight for %.1f seconds\n', ...
                            CFG.LOOP_RECOVERY_DURATION);
                end
            end
        end
        
        % Update previous pose and time for next iteration
        prev_pose = curr_pose;
        prev_time = curr_time;
        
        % --- Calculate Distance to Goal ---
        dx = goal.x - x;
        dy = goal.y - y;
        dist_to_goal = sqrt(dx^2 + dy^2);
        heading_error_to_goal = wrapToPi(goal.theta - theta);
        
        % --- Check if Goal Reached ---
        if (dist_to_goal < CFG.GOAL_POSITION_TOL) && ...
           (abs(heading_error_to_goal) < CFG.GOAL_HEADING_TOL)
            goal_reached = true;
            fprintf('\n✓✓✓ GOAL REACHED! ✓✓✓\n');
            fprintf('  Final position error: %.3f m\n', dist_to_goal);
            fprintf('  Final heading error: %.1f°\n', rad2deg(heading_error_to_goal));
            break;
        end
        
        % --- Control Mode Selection: Recovery or Pure Pursuit ---
        if recovery_mode
            % --- RECOVERY MODE: Drive Straight ---
            % Check if recovery period is over
            if (curr_time - recovery_start_time) >= CFG.LOOP_RECOVERY_DURATION
                recovery_mode = false;
                cumulative_heading_change = 0;  % Reset loop detection
                fprintf('  << Recovery complete, resuming Pure Pursuit\n');
            end
            
            % Drive straight at current heading (no turning)
            v_cmd = CFG.V_DESIRED;
            omega = 0;  % Zero angular velocity = straight line!
            
            % Set dummy values for logging
            lookahead_x = x + 0.5 * cos(theta);
            lookahead_y = y + 0.5 * sin(theta);
            crosstrack_error = NaN;  % Not tracking path during recovery
            heading_error = 0;
            
        else
            % --- NORMAL MODE: Pure Pursuit Path Tracking ---
            % Find the lookahead point on the planned path
            [lookahead_x, lookahead_y, ~, crosstrack_error] = ...
                findLookaheadPoint(x, y, path, CFG.LOOKAHEAD_DISTANCE);
            
            % Calculate angle to lookahead point
            alpha = atan2(lookahead_y - y, lookahead_x - x);
            
            % Calculate heading error (how much to turn)
            heading_error = wrapToPi(alpha - theta);
            
            % Pure Pursuit control law
            % Curvature = 2*sin(heading_error) / lookahead_distance
            curvature = (2 * sin(heading_error)) / CFG.LOOKAHEAD_DISTANCE;
            
            % Convert to angular velocity: ω = v * κ
            omega = CFG.V_DESIRED * curvature;
            
            % Apply limits
            omega = max(-CFG.MAX_ANGULAR_VEL, min(CFG.MAX_ANGULAR_VEL, omega));
            v_cmd = CFG.V_DESIRED;
        end
        
        % --- Send Command to LIMO ---
        % Format: 'v,w' where v=linear [m/s], w=angular [rad/s]
        cmd_str = sprintf('%.2f,%.2f', v_cmd, omega);
        write(tcp, uint8(cmd_str));
        
        % --- Update Visualization ---
        set(h_robot, 'XData', x, 'YData', y);
        
        traj_x(end+1) = x;
        traj_y(end+1) = y;
        set(h_trajectory, 'XData', traj_x, 'YData', traj_y);
        
        set(h_lookahead, 'XData', lookahead_x, 'YData', lookahead_y);
        
        heading_vec_x = 0.3 * cos(theta);
        heading_vec_y = 0.3 * sin(theta);
        set(h_heading, 'XData', x, 'YData', y, ...
                       'UData', heading_vec_x, 'VData', heading_vec_y);
        
        % Update status text
        t = toc(start_time);
        if recovery_mode
            % Show RECOVERY MODE status
            status_str = sprintf(['**RECOVERY MODE**\n' ...
                                  'Time: %.1f s | Iter: %d\n' ...
                                  'Pos: (%.2f, %.2f) m\n' ...
                                  'Heading: %.1f°\n' ...
                                  'Dist to goal: %.2f m\n' ...
                                  'Driving STRAIGHT\n' ...
                                  'v: %.2f m/s | ω: 0°/s'], ...
                                  t, loop_count, x, y, rad2deg(theta), ...
                                  dist_to_goal, v_cmd);
        else
            % Normal Pure Pursuit status
            status_str = sprintf(['Time: %.1f s | Iter: %d\n' ...
                                  'Pos: (%.2f, %.2f) m\n' ...
                                  'Heading: %.1f°\n' ...
                                  'Dist to goal: %.2f m\n' ...
                                  'XTE: %.3f m\n' ...
                                  'v: %.2f m/s\n' ...
                                  'ω: %.1f °/s'], ...
                                  t, loop_count, x, y, rad2deg(theta), ...
                                  dist_to_goal, crosstrack_error, ...
                                  v_cmd, rad2deg(omega));
        end
        set(h_text, 'String', status_str);
        
        drawnow limitrate;
        
        % --- Log Data ---
        log.time(end+1) = t;
        log.x(end+1) = x;
        log.y(end+1) = y;
        log.theta(end+1) = theta;
        log.v_cmd(end+1) = v_cmd;
        log.omega_cmd(end+1) = omega;
        log.crosstrack_error(end+1) = crosstrack_error;
        log.heading_error(end+1) = heading_error;
        
        % --- Print Status (every second) ---
        if mod(loop_count, CFG.CONTROL_RATE) == 0
            fprintf('t=%.1fs | Pos=(%.2f,%.2f,%.0f°) | Dist=%.2fm | XTE=%.3fm | v=%.2f ω=%.1f°/s\n', ...
                    t, x, y, rad2deg(theta), dist_to_goal, crosstrack_error, ...
                    v_cmd, rad2deg(omega));
        end
        
        % --- Safety Timeout ---
        if t > CFG.MAX_TIME
            fprintf('\n✗ TIMEOUT after %.0f seconds\n', CFG.MAX_TIME);
            break;
        end
        
        % --- Maintain Loop Timing ---
        elapsed = toc(loop_start);
        if elapsed < CFG.dt
            pause(CFG.dt - elapsed);
        end
    end
    
    %% ===============================================
    %% SUMMARY
    %% ===============================================
    
    fprintf('\n========================================\n');
    fprintf('         EXECUTION SUMMARY\n');
    fprintf('========================================\n');
    
    if ~isempty(log.time)
        fprintf('Total time: %.1f s\n', log.time(end));
        fprintf('Control loops: %d\n', loop_count);
        fprintf('Average speed: %.3f m/s\n', mean(log.v_cmd));
        fprintf('\nTracking Errors:\n');
        fprintf('  RMS crosstrack: %.3f m\n', sqrt(mean(log.crosstrack_error.^2)));
        fprintf('  Mean crosstrack: %.3f m\n', mean(abs(log.crosstrack_error)));
        fprintf('  Max crosstrack: %.3f m\n', max(abs(log.crosstrack_error)));
        
        if goal_reached
            fprintf('\nFinal errors:\n');
            fprintf('  Position: %.3f m\n', dist_to_goal);
            fprintf('  Heading: %.1f°\n', rad2deg(heading_error_to_goal));
        end
        
        % --- Performance Plots ---
        figure('Name', 'Performance Analysis', 'Position', [100 100 1200 800]);
        
        subplot(3,1,1);
        plot(log.time, log.crosstrack_error*100, 'b-', 'LineWidth', 1.5);
        grid on;
        xlabel('Time [s]');
        ylabel('Crosstrack Error [cm]');
        title('Distance from Dubins Path');
        yline(0, 'k--');
        
        subplot(3,1,2);
        plot(log.time, rad2deg(log.heading_error), 'r-', 'LineWidth', 1.5);
        grid on;
        xlabel('Time [s]');
        ylabel('Heading Error [°]');
        title('Heading Error to Lookahead');
        yline(0, 'k--');
        
        subplot(3,1,3);
        yyaxis left;
        plot(log.time, log.v_cmd, 'g-', 'LineWidth', 1.5);
        ylabel('Linear Vel [m/s]');
        yyaxis right;
        plot(log.time, rad2deg(log.omega_cmd), 'm-', 'LineWidth', 1.5);
        ylabel('Angular Vel [°/s]');
        xlabel('Time [s]');
        title('Velocity Commands');
        grid on;
    end
    
    fprintf('========================================\n\n');
    
catch ME
    fprintf('\n✗ ERROR: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('  In: %s (line %d)\n\n', ME.stack(1).name, ME.stack(1).line);
    end
end

%% ===============================================
%% CLEANUP
%% ===============================================

fprintf('Cleaning up...\n');

% Stop robot
if ~isempty(tcp) && isvalid(tcp)
    write(tcp, uint8('0.00,0.00'));
    pause(0.5);
    clear tcp;
    fprintf('  ✓ LIMO stopped\n');
end

% Disconnect MQTT
if ~isempty(mqttClient)
    clear mqttClient;
    fprintf('  ✓ MQTT disconnected\n');
end

fprintf('\nScript complete.\n\n');

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
%   mqttClient - MQTT client
%   limoNum - LIMO number string
%   CFG - Configuration struct
%   prev_pose - Previous pose [x, y, theta] (for velocity calculation)
%   dt - Time step since last reading [s]
%
% OUTPUTS:
%   pose - [x, y, theta] where theta is from velocity direction
%   valid - Boolean if data is good

    pose = [0, 0, 0];
    valid = false;
    
    try
        % --- Read Position ---
        posTable = read(mqttClient, 'Topic', sprintf("rb/limo%s/pos", limoNum));
        
        if isempty(posTable)
            return;
        end
        
        rawData = posTable.Data{end};
        rawData = erase(rawData, ["[", "]"]);
        numericPos = str2double(split(rawData, ","))';
        
        if any(isnan(numericPos))
            return;
        end
        
        % Transform: MoCap [X, Z, Y] to Map [x, y]
        mocap_x = numericPos(1);
        mocap_y = numericPos(3);  % Y is 3rd element!
        
        x = mocap_x - CFG.MOCAP_ORIGIN_X;
        y = -(mocap_y - CFG.MOCAP_ORIGIN_Y);  % Negated!
        
        % --- Calculate Heading from Velocity Direction ---
        if ~isempty(prev_pose) && dt > 0
            % Calculate velocity vector
            dx = x - prev_pose(1);  % Change in X [m]
            dy = y - prev_pose(2);  % Change in Y [m]
            
            % Calculate speed
            speed = sqrt(dx^2 + dy^2) / dt;  % [m/s]
            
            % Only update heading if robot is moving
            % (avoids noise when stationary)
            speed_threshold = 0.05;  % Minimum speed to update heading [m/s]
            
            if speed > speed_threshold
                % Heading = direction of motion
                theta = atan2(dy, dx);  % [rad]
            else
                % Robot is stationary - keep previous heading
                theta = prev_pose(3);
            end
        else
            % First reading - read from rotation data as initial heading
            rotTable = read(mqttClient, 'Topic', sprintf("rb/limo%s/rot", limoNum));
            
            if ~isempty(rotTable)
                rawRot = rotTable.Data{end};
                rawRot = erase(rawRot, ["[", "]"]);
                rot_values = str2double(split(rawRot, ","))';
                
                if ~any(isnan(rot_values)) && length(rot_values) >= 3
                    % Use yaw from rotation data as initial heading
                    theta = -rot_values(3);  % Negate to match Y-axis flip
                else
                    theta = 0;  % Default to facing East
                end
            else
                theta = 0;  % Default to facing East
            end
        end
        
        pose = [x, y, theta];
        valid = true;
        
    catch
        % Error in data read
    end
end

%% ===============================================
%% HELPER FUNCTION: FIND LOOKAHEAD POINT
%% ===============================================

function [lookahead_x, lookahead_y, lookahead_idx, crosstrack_error] = ...
    findLookaheadPoint(robot_x, robot_y, path, lookahead_distance)
% FINDLOOKAHEADPOINT - Find point on path at lookahead distance
%
% INPUTS:
%   robot_x, robot_y - Current position
%   path - Structure with .x, .y, .s (arc length)
%   lookahead_distance - Distance ahead to look
%
% OUTPUTS:
%   lookahead_x, lookahead_y - Lookahead coordinates
%   lookahead_idx - Index in path
%   crosstrack_error - Distance from path

    % Find closest point on path
    distances = sqrt((path.x - robot_x).^2 + (path.y - robot_y).^2);
    [crosstrack_error, closest_idx] = min(distances);
    
    % Arc length at closest point
    s_closest = path.s(closest_idx);
    
    % Desired arc length (lookahead ahead)
    s_lookahead = s_closest + lookahead_distance;
    
    % Find point at lookahead arc length
    if s_lookahead > path.s(end)
        lookahead_idx = length(path.x);  % Use goal
    else
        lookahead_idx = find(path.s >= s_lookahead, 1, 'first');
        if isempty(lookahead_idx)
            lookahead_idx = length(path.x);
        end
    end
    
    lookahead_x = path.x(lookahead_idx);
    lookahead_y = path.y(lookahead_idx);
end