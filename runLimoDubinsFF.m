function runLimoDubinsFF_final()
% RUNLIMODUBINSFF_FINAL - Feedforward control for Limo using TCP
% Uses dubinsConnection with robust interpolation and error handling.

%% ---------------- Parameters ----------------
v_max = 0.3;       % max linear velocity [m/s]
dt = 0.55;         % feedforward interval [s]
R_min = 0.4;       % minimum turning radius [m]
fixedIP = '192.168.1.';  % first 3 octets of Limo IP

%% ---------------- User Input ----------------
disp('Enter start pose [x, y, theta] in meters and degrees:');
x0 = input('Start x (m): ');
y0 = input('Start y (m): ');
theta0_deg = input('Start theta (deg): ');
disp('Enter goal pose [x, y, theta] in meters and degrees:');
xf = input('Goal x (m): ');
yf = input('Goal y (m): ');
thetaf_deg = input('Goal theta (deg): ');
lastDigits = input('Enter last 3 digits of Limo IP: ','s');

% Convert degrees to radians
theta0 = deg2rad(theta0_deg);
thetaf_rad = deg2rad(thetaf_deg); % Use a temporary variable for the original goal angle

limoIP = [fixedIP, lastDigits];
limoPort = 12345;
startPose = [x0, y0, theta0];

% Normalize the goal angle to [-pi, pi] for robust path planning
thetaf = wrapToPi(thetaf_rad);
fprintf('Goal theta normalized from %.4f rad to %.4f rad for planning.\n', thetaf_rad, thetaf);
goalPose  = [xf, yf, thetaf];

%% ---------------- Path Planning and Execution ----------------
[path_points, num_steps] = dubinsPathPlanning(startPose, goalPose, R_min, dt, v_max);

if isempty(path_points)
    disp('Script stopped due to path planning failure.');
    return;
end

tcpClient = tcpConnection(limoIP, limoPort);
if isempty(tcpClient)
    disp('Script stopped due to connection failure.');
    return;
end

% Extract path coordinates and orientation
path_x = path_points(:, 1);
path_y = path_points(:, 2);
path_theta = path_points(:, 3);

% Feedforward Loop
fprintf('t = %.2f s | x_d = %.2f, y_d = %.2f, theta_d = %.2f | v = %.2f, omega = %.2f\n', ...
        0, path_x(1), path_y(1), path_theta(1), 0, 0); % Initial state
    
for k = 1:num_steps - 1
    t = k * dt;
    
    % Current desired pose
    x_d = path_x(k);
    y_d = path_y(k);
    theta_d = path_theta(k);
    
    % Next pose (for finite difference calculation)
    x_next = path_x(k+1);
    y_next = path_y(k+1);
    theta_next = path_theta(k+1);

    sendFeedforwardCommand(tcpClient, x_d, y_d, theta_d, x_next, y_next, theta_next, dt, v_max, t);
    
    pause(dt);
end

% Send final stop command (k=num_steps)
cmd = sprintf('0.00,0.00');
write(tcpClient, uint8(cmd));

%% ---------------- Cleanup ----------------
clear tcpClient
disp('Feedforward path complete. TCP connection closed.');

end % end of runLimoDubinsFF_final

%% ---------------- Local Functions ----------------

function [path_points, num_steps] = dubinsPathPlanning(startPose, goalPose, R_min, dt, v_max)
    % DUBINSPATHPLANNING Calculates the path points and number of steps.
    
    dub = dubinsConnection('MinTurningRadius', R_min);
    
    % The 'connect' function sometimes returns a 1x1 cell array, we unwrap it.
    dubPath_raw = connect(dub, startPose, goalPose);
    
    % Check if a path was found and unwrap the cell array
    if isempty(dubPath_raw) || ~iscell(dubPath_raw)
        error('Dubins path failed. Try increasing R_min or adjusting poses.');
    end
    
    dubPath = dubPath_raw{1};

    % --- Step 1: Manual Length Calculation (Highly Robust) ---
    % Since .Length may fail and pathLength() may not exist, we calculate length manually
    % by interpolating many points and summing the Euclidean distance between them.
    temp_points = interpolate(dubPath, linspace(0, 1, 1000));
    path_length = sum(vecnorm(diff(temp_points(:, 1:2)), 2, 2));

    % --- Step 2: Determine Step Count and Final Interpolation ---
    % Use the actual path length to determine a dense sampling for feedforward control.
    % Add a buffer (+10) to ensure the final rotation is fully captured.
    num_steps = ceil(path_length / (v_max * dt)) + 10;

    % Create distance vector for final interpolation
    path_s = linspace(0, path_length, num_steps);

    % Interpolate path_points along the Dubins path (Nx3 array)
    path_points = interpolate(dubPath, path_s);
    
    disp('Dubins path generated successfully using robust length calculation.');
    
    % --- Path Visualization (Debug) ---
    figure('Name', 'Dubins Path Visualization');
    hold on;
    % Plot the path
    plot(path_points(:, 1), path_points(:, 2), 'b-', 'LineWidth', 2);
    
    % Plot start pose (green circle, line indicates orientation)
    quiver(startPose(1), startPose(2), cos(startPose(3)), sin(startPose(3)), 0.5, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    plot(startPose(1), startPose(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    text(startPose(1), startPose(2), 'Start', 'VerticalAlignment', 'bottom');
    
    % Plot goal pose (red circle, line indicates orientation)
    quiver(goalPose(1), goalPose(2), cos(goalPose(3)), sin(goalPose(3)), 0.5, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    plot(goalPose(1), goalPose(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    text(goalPose(1), goalPose(2), 'Goal', 'VerticalAlignment', 'bottom');
    
    title('Planned Dubins Path');
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal;
    grid on;
    hold off;
end

function tcpClient = tcpConnection(limoIP, limoPort)
    % TCPCONNECTION Establishes the TCP connection.
    tcpClient = []; % Initialize to empty array
    try
        tcpClient = tcpclient(limoIP, limoPort, 'Timeout', 5); % 5 second timeout
        disp(['Connected to Limo ', limoIP, ' via TCP/IP.']);
    catch ME
        fprintf('Warning: Attempted to connect to: %s:%d\n', limoIP, limoPort);
        error('Unable to connect. Check IP, WiFi, and TCP server running on Limo. Underlying error: %s', ME.message);
    end
end

function sendFeedforwardCommand(tcpClient, x_d, y_d, theta_d, x_next, y_next, theta_next, dt, v_max, t)
    % SENDFEEDFORWARDCOMMAND Computes and sends one feedforward command.

    % Compute linear velocity (finite difference for position)
    dx = (x_next - x_d) / dt;
    dy = (y_next - y_d) / dt;
    v_d = sqrt(dx^2 + dy^2);
    
    % Compute angular velocity (finite difference for orientation using angdiff)
    % Use angdiff to correctly handle wrap-around angles (e.g., pi to -pi)
    dtheta = angdiff(theta_d, theta_next); 
    omega_d = dtheta / dt;
    
    % Clamp linear velocity to maximum allowed
    v_d = min(v_d, v_max);
    
    % Send command over TCP
    cmd = sprintf('%.2f,%.2f', v_d, omega_d);
    write(tcpClient, uint8(cmd));
    
    % Display debug info
    fprintf('t = %.2f s | x_d = %.2f, y_d = %.2f, theta_d = %.2f | v = %.2f, omega = %.2f\n', ...
        t, x_d, y_d, theta_d, v_d, omega_d);
end