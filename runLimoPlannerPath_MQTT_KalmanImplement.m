function runLimoPlannerPath_MQTT_KalmanImplement()
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

valid_route = false;
while ~valid_route
    route_input = input('Select route (1, 2, or 3): ', 's');
    route_num = str2double(route_input);
    if ismember(route_num, [1, 2, 3])
        valid_route = true;
    else
        fprintf('Invalid input. Please enter 1, 2, or 3.\n');
    end
end

fprintf('\n--- LIMO Network Configuration ---\n');
fprintf('Example: Enter "101" for 192.168.1.101\n');
LIMO_IP_LAST_3 = input('IP last 3 digits: ', 's');
fprintf('\nEnter LIMO identification number (e.g. "777")\n');
LIMO_NUMBER = input('LIMO number: ', 's');

%% ===============================================
%% ROUTE-SPECIFIC CONFIGURATION
%% ===============================================
switch route_num
    case 1
        % Route 1: Two-column obstacle course
        Goal = [5.0; 4.5];
        rx_temp = [5.0, 4.5, 4.0, 4.0, 4.0, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0];
        ry_temp = [4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 2.0, 2.5, 3.0, 3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0];
        rx = fliplr(rx_temp); ry = fliplr(ry_temp);
        obs_xy = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5;
                  0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0];
    case 2
        % Route 2: 3x3 grid
        Goal = [5.0; 4.5];
        rx = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 3.5, 4.0, 4.5, 5.0];
        ry = [0.0, 0.0, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5];
        obs_xy = [1.0, 2.5, 4.0, 0.0, 1.5, 3.0, 1.0, 2.5, 4.0;
                  1.0, 1.0, 1.0, 2.5, 2.5, 2.5, 4.0, 4.0, 4.0];
    case 3
        % Route 3: Complex S-shaped path
        Goal = [0.0; 1.5];
        rx = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0];
        ry = [0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 0.5, 1.0, 1.0, 2.0, 2.5, 3.0, 3.5, 4.0, 4.0, 4.0, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5];
        obs_xy = [0.0, 0.5, 1.0, 1.0, 1.0, 1.5, 2.0, 2.5, 2.5, 2.5, 2.5, 3.0, 3.5, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 0.0, 0.5, 1.0, 2.5, 2.5;
                  1.0, 1.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 2.5, 3.0, 3.5, 3.5, 3.5, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 3.5, 3.5, 3.5, 0.0, 0.5];
end

%% ===============================================
%% CONFIGURATION & TUNING NOTES
%% ===============================================

CFG.limo_ip_prefix = '192.168.1.';
CFG.limo_port = 12345;
CFG.mqtt_broker = 'mqtt://rasticvm.lan';
CFG.MOCAP_ORIGIN_X = -4.5;
CFG.MOCAP_ORIGIN_Y = 2.7;

% --- Controller Parameters (Tuned) ---
% TUNING NOTE: Lookahead Distance
% Value: 0.5m. Origin: Found experimentally.
% Sensitivity: <0.3m oscillates; >0.8m under-steers.
CFG.LOOKAHEAD_DISTANCE = 0.5; 

% TUNING NOTE: Desired Velocity
% Value: 0.25 m/s. Origin: Safety.
CFG.V_DESIRED = 0.25; 

% TUNING NOTE: Minimum Turning Radius
% Value: 0.25m. Origin: Ackerman steering limit.
CFG.MIN_TURNING_RADIUS = 0.25;

% Safety and Timing
CFG.LOOP_RECOVERY_DURATION = 1.0;
CFG.LOOPS_BEFORE_RECOVERY = 2;
CFG.GOAL_POSITION_TOL = 0.10;
CFG.MAX_LINEAR_VEL = 0.3;
CFG.MAX_ANGULAR_VEL = deg2rad(60);
CFG.MAX_TIME = 360;
CFG.CONTROL_RATE = 20;
CFG.dt = 1/CFG.CONTROL_RATE;

tcp = []; mqttClient = [];

try
    %% ===============================================
    %% PREPARE PATH & EKF
    %% ===============================================
    fprintf('Validating planner path...\n');
    N = length(rx);
    waypoints = zeros(N,3);
    for i = 1:N-1
        dx = rx(i+1) - rx(i); dy = ry(i+1) - ry(i);
        waypoints(i,:) = [rx(i), ry(i), atan2(dy, dx)];
    end
    waypoints(N,:) = [rx(N), ry(N), waypoints(N-1,3)];

    % EKF Initialization
    fprintf('Initializing Extended Kalman Filter...\n');
    Q = diag([0.005, 0.005, 0.001]); 
    R = diag([0.0001, 0.0001, 0.0001]); 
    last_u = [0; 0]; 

    % MQTT Connection
    fprintf('Connecting to MQTT broker: %s\n', CFG.mqtt_broker);
    mqttClient = mqttclient(CFG.mqtt_broker);
    subscribe(mqttClient, sprintf("rb/limo%s", LIMO_NUMBER));

    % Initial Pose
    fprintf('Waiting for MoCap data...\n');
    for i = 1:10
        [test_pose, valid] = getRobotPose_MQTT(mqttClient, LIMO_NUMBER, CFG, [], 0);
        if valid, fprintf('.'); break; end
        pause(1);
    end
    [start_pose, valid] = getRobotPose_MQTT(mqttClient, LIMO_NUMBER, CFG, [], 0);
    if ~valid, error('Could not get initial MoCap lock.'); end
    
    % Smooth Start: Prepend current position to path
    rx = [start_pose(1), rx];
    ry = [start_pose(2), ry];
    N = length(rx); 
    waypoints = zeros(N,3);
    for i = 1:N-1
        dx = rx(i+1) - rx(i); dy = ry(i+1) - ry(i);
        waypoints(i,:) = [rx(i), ry(i), atan2(dy, dx)];
    end
    waypoints(N,:) = [rx(N), ry(N), waypoints(N-1,3)];
    
    x_hat_prev = start_pose'; 
    P_prev = eye(3) * 0.1;    

    % TCP Connection
    limo_ip = [CFG.limo_ip_prefix LIMO_IP_LAST_3];
    fprintf('\nConnecting to LIMO TCP: %s:%d\n', limo_ip, CFG.limo_port);
    tcp = tcpclient(limo_ip, CFG.limo_port, 'Timeout', 5);
    write(tcp, uint8('0.00,0.00')); pause(0.5);

    % Dubins Path
    dubinsPathFull = [];
    dubinsObj = dubinsConnection('MinTurningRadius', CFG.MIN_TURNING_RADIUS);
    for i = 1:N-1
        [segObj,~] = connect(dubinsObj, waypoints(i,:), waypoints(i+1,:));
        if ~isempty(segObj)
            segPoints = interpolate(segObj{1}, 0:0.05:segObj{1}.Length);
            if i>1, segPoints = segPoints(2:end,:); end
            dubinsPathFull = [dubinsPathFull; segPoints];
        end
    end
    path.x = dubinsPathFull(:,1); path.y = dubinsPathFull(:,2);
    path.s = [0; cumsum(sqrt(diff(path.x).^2 + diff(path.y).^2))];

    % Visualization
    figure('Name','LIMO Path Following','Position',[100 100 1000 800]);
    plot(obs_xy(1,:), obs_xy(2,:), 'ro', 'MarkerFaceColor', 'r'); hold on; grid on; axis equal;
    plot(rx, ry, 'b-o', 'LineWidth', 1.5);
    plot(path.x, path.y, 'c-', 'LineWidth', 2);
    plot(Goal(1), Goal(2), 'g*', 'MarkerSize', 20);
    robot_plot = plot(NaN, NaN, 'ko', 'MarkerFaceColor', 'y');
    lookahead_plot = plot(NaN, NaN, 'mo', 'MarkerFaceColor', 'm');
    title(sprintf('LIMO %s - Route %d', LIMO_NUMBER, route_num));

    %% ===============================================
    %% CONTROL LOOP
    %% ===============================================
    fprintf('\nStarting control loop...\n');
    start_time = tic;
    loop_history = []; loop_counter = 0; recovery_start_time = [];
    
    % --- LOGGING VARIABLES FOR COMPARISON PLOT ---
    log_raw = [];   % Stores [time, x, y, theta] (Raw Sensor)
    log_ekf = [];   % Stores [time, x, y, theta] (Kalman Filter)
    
    while true
        loop_start = tic;
        
        % 1. Get Raw Measurement
        [raw_pose, valid] = getRobotPose_MQTT(mqttClient, LIMO_NUMBER, CFG, x_hat_prev', CFG.dt);
        z_k = raw_pose'; % [x; y; theta]
        
        % 2. Log Raw Data
        curr_time = toc(start_time);
        if valid
            log_raw = [log_raw; curr_time, z_k'];
        else
            % Log NaNs to show data loss in plot
            log_raw = [log_raw; curr_time, NaN, NaN, NaN];
            z_k = x_hat_prev; % Fallback for EKF
        end

        % 3. Kalman Filter Step
        [x_hat_new, P_new] = kalmanFilter(x_hat_prev, P_prev, last_u, z_k, CFG.dt, Q, R);
        
        % 4. Log Filtered Data & Update State
        log_ekf = [log_ekf; curr_time, x_hat_new'];
        x_hat_prev = x_hat_new;
        P_prev = P_new;
        
        % 5. Use FILTERED State for Control
        curr_x = x_hat_new(1);
        curr_y = x_hat_new(2);
        curr_theta = x_hat_new(3);
        
        % 6. Goal Check
        if sqrt((curr_x - Goal(1))^2 + (curr_y - Goal(2))^2) < CFG.GOAL_POSITION_TOL
            fprintf('\n✓ Goal reached! Time: %.1f s\n', curr_time);
            write(tcp, uint8('0.00,0.00')); break;
        end
        
        % 7. Pure Pursuit Control
        % See diagram below for geometry
        [lookahead_x, lookahead_y, ~, crosstrack_error] = ...
            findLookaheadPoint(curr_x, curr_y, path, CFG.LOOKAHEAD_DISTANCE);
            
        % Loop Recovery
        loop_history = [loop_history, crosstrack_error];
        if length(loop_history) > 30, loop_history = loop_history(end-29:end); end
        if length(loop_history) >= 30 && sum(diff(sign(loop_history)) ~= 0) >= 20
            loop_counter = loop_counter + 1;
            if loop_counter >= CFG.LOOPS_BEFORE_RECOVERY && isempty(recovery_start_time)
                recovery_start_time = tic; loop_counter = 0; loop_history = [];
            end
        end

        if ~isempty(recovery_start_time)
            if toc(recovery_start_time) < CFG.LOOP_RECOVERY_DURATION
                v_cmd = 0.0; w_cmd = deg2rad(30);
            else
                recovery_start_time = [];
            end
        else
            dx = lookahead_x - curr_x; dy = lookahead_y - curr_y;
            desired_theta = atan2(dy, dx);
            theta_error = atan2(sin(desired_theta - curr_theta), cos(desired_theta - curr_theta));
            lookahead_dist = sqrt(dx^2 + dy^2);
            
            % Pure Pursuit Control Law (Coulter 1992)
            w_cmd = (2 * CFG.V_DESIRED * sin(theta_error)) / lookahead_dist;
            
            if abs(theta_error) < deg2rad(90)
                v_cmd = CFG.V_DESIRED * cos(theta_error);
            else
                v_cmd = CFG.V_DESIRED * 0.3;
            end
        end
        
        v_cmd = max(0, min(v_cmd, CFG.MAX_LINEAR_VEL));
        w_cmd = max(-CFG.MAX_ANGULAR_VEL, min(w_cmd, CFG.MAX_ANGULAR_VEL));
        
        write(tcp, uint8(sprintf('%.2f,%.2f', v_cmd, w_cmd)));
        last_u = [v_cmd; w_cmd];

        set(robot_plot, 'XData', curr_x, 'YData', curr_y);
        set(lookahead_plot, 'XData', lookahead_x, 'YData', lookahead_y);
        drawnow limitrate;
        
        elapsed = toc(loop_start);
        if elapsed < CFG.dt, pause(CFG.dt - elapsed); end
        if curr_time > CFG.MAX_TIME, break; end
    end
    
    %% ===============================================
    %% PLOTTING: RAW vs KALMAN COMPARISON
    %% ===============================================
    if ~isempty(log_raw) && ~isempty(log_ekf)
        figure('Name', 'Kalman Filter Performance', 'Position', [100, 100, 1200, 500]);

        % Plot 1: Trajectory Map
        subplot(1, 2, 1);
        plot(log_raw(:,2), log_raw(:,3), 'ro', 'MarkerSize', 5, 'LineWidth', 0.5, 'DisplayName', 'Raw (Noisy)');
        hold on;
        plot(log_ekf(:,2), log_ekf(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'EKF (Smooth)');
        grid on; axis equal; legend('Location', 'best');
        xlabel('X [m]'); ylabel('Y [m]'); title('Trajectory Smoothing');

        % Plot 2: X-Position vs Time
        subplot(1, 2, 2);
        plot(log_raw(:,1), log_raw(:,2), 'ro', 'MarkerSize', 4);
        hold on;
        plot(log_ekf(:,1), log_ekf(:,2), 'b-', 'LineWidth', 1.5);
        grid on; legend('Raw', 'EKF');
        xlabel('Time [s]'); ylabel('X Position [m]'); title('Noise Reduction (X-Axis)');
        
        fprintf('\nComparison plots generated.\n');
    else
        fprintf('\nWARNING: Log buffers empty. Robot likely did not run or no valid data received.\n');
    end

catch ME
    fprintf('Error: %s\n', ME.message);
    if ~isempty(ME.stack), fprintf('Line: %d\n', ME.stack(1).line); end
end

if ~isempty(tcp), write(tcp, uint8('0.00,0.00')); clear tcp; end
if ~isempty(mqttClient), clear mqttClient; end
end

%% ===============================================
%% HELPER FUNCTIONS
%% ===============================================

function [pose, valid] = getRobotPose_MQTT(mqttClient, limoNum, CFG, prev_pose, dt)
    pose = [0, 0, 0]; valid = false;
    try
        % --- FIXED: Read the message BEFORE checking its height! ---
        mqttMsg = peek(mqttClient); 
        
        if isempty(mqttMsg), return; end
        
        if height(mqttMsg) > 1
            mqttMsg = mqttMsg(end, :);
        end
        
        expected_topic = sprintf('rb/limo%s', limoNum);
        if ~strcmp(char(mqttMsg.Topic), expected_topic), return; end
        
        % Check if Data is cell or string
        dataStr = char(mqttMsg.Data);
        if iscell(dataStr), dataStr = dataStr{1}; end
        
        jsonData = jsondecode(dataStr);
        if ~isfield(jsonData, 'pos'), return; end
        
        x = jsonData.pos(1) - CFG.MOCAP_ORIGIN_X;
        y = -(jsonData.pos(3) - CFG.MOCAP_ORIGIN_Y); 
        
        if ~isempty(prev_pose) && dt > 0
            dx = x - prev_pose(1); dy = y - prev_pose(2);
            speed = sqrt(dx^2 + dy^2) / dt;
            if speed > 0.05, theta = atan2(dy, dx); else, theta = prev_pose(3); end
        else
            theta = -jsonData.rot(3);
        end
        pose = [x, y, theta]; valid = true;
    catch
    end
end

function [lookahead_x, lookahead_y, lookahead_idx, crosstrack_error] = ...
    findLookaheadPoint(robot_x, robot_y, path, lookahead_distance)
    
    distances = sqrt((path.x - robot_x).^2 + (path.y - robot_y).^2);
    [crosstrack_error, closest_idx] = min(distances);
    
    s_lookahead = path.s(closest_idx) + lookahead_distance;
    lookahead_idx = find(path.s >= s_lookahead, 1, 'first');
    if isempty(lookahead_idx), lookahead_idx = length(path.x); end
    
    lookahead_x = path.x(lookahead_idx);
    lookahead_y = path.y(lookahead_idx);
end

