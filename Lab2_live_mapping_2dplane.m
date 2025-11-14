clearvars;
%%
mqttClient = mqttclient("mqtt://rasticvm.lan"); %connect to mqtt broker
%%
SubscrTable = subscribe(mqttClient,"rb/limo780/rot");
SubscrTable = subscribe(mqttClient,"rb/limo780/pos");
%SubscrTable = subscribe(mqttClient, mqttTopic, Name=values);
%%
% ===== CONTINUOUS MOTION CAPTURE TRACKING (2D TOP-DOWN VIEW) =====
% This script tracks a robot's X-Y position in real-time using MQTT data
% Z-elevation data is ignored for a clean top-down view
% Runs continuously until stopped by user

% ===== SAMPLING CONFIGURATION =====
Ts = 1/120;     % Sampling period in seconds (8.33ms)
                % Matches the 120 Hz transmission rate of the motion capture system
                % This ensures we don't miss any position updates

dataLog = [];   % Initialize empty array to store all position data over time
                % Format: [time, X, Z_elevation, Y] - one row per sample
                % Even though we plot 2D, we store all 3 coordinates for later analysis

% ===== CREATE FIGURE AND STOP BUTTON =====
fig = figure('Name', 'Live Robot Tracking (2D Top View) - Press Stop to End', ...
             'NumberTitle', 'off');  % Create new figure window with custom title

% Create red STOP button in bottom-left corner of figure
stopButton = uicontrol('Style', 'pushbutton', ...          % Button UI element
                       'String', 'STOP', ...               % Button text
                       'FontSize', 14, ...                 % Large font for visibility
                       'FontWeight', 'bold', ...           % Bold text
                       'BackgroundColor', [1 0.3 0.3], ... % Red background
                       'Position', [20 20 100 40], ...     % [left bottom width height] in pixels
                       'Callback', @(src,evt) set(src, 'UserData', true));  % When clicked, sets UserData to true

% ===== INITIALIZE 2D SCATTER PLOT =====
h = scatter(NaN, NaN, 150, ...              % Create scatter plot with no data initially (NaN, NaN)
            'filled', ...                    % Filled circle marker
            'MarkerFaceColor', 'r', ...      % Red fill color
            'MarkerEdgeColor', 'k', ...      % Black edge/outline
            'LineWidth', 1.5);               % Thickness of the marker edge

hold on;  % Keep the plot so we can add more elements without erasing

grid on;  % Turn on grid lines for easier position reading

% Label the axes with descriptive names and formatting
xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');  % Horizontal axis (left/right)
ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');  % Horizontal axis (forward/back)

% Add title explaining what we're viewing
title('Live Robot Position - Top-Down View (X-Y Plane)', 'FontSize', 14);

axis equal;  % Make sure 1 meter in X = 1 meter in Y visually (no distortion)

% Set initial axis limits (will auto-adjust as robot moves)
xlim([-1 1]);  % Start with -1m to +1m in X direction
ylim([-1 1]);  % Start with -1m to +1m in Y direction

% ===== ADD TEXT ANNOTATIONS =====
% Create text object to display current coordinates above the robot marker
posText = text(0, 0, '', ...                           % Initialize at origin (0,0) with empty string
               'FontSize', 10, ...                     % Font size for readability
               'Color', 'blue', ...                    % Blue text color
               'FontWeight', 'bold', ...               % Bold for emphasis
               'VerticalAlignment', 'bottom', ...      % Text appears above the position
               'HorizontalAlignment', 'center');       % Text centered on position

% Create info text explaining coordinate mapping from MQTT to plot
infoText = text(0, 0, 'MQTT: [X, Z, Y] → Plot: [X, Y]', ...  % Explain the transformation
                'Units', 'normalized', ...             % Use normalized coordinates (0-1)
                'Position', [0.02, 0.95], ...          % Top-left corner of plot area
                'FontSize', 9, ...                     % Smaller font
                'Color', 'black', ...                  % Black text
                'BackgroundColor', 'yellow');          % Yellow highlight box

% Print startup messages to command window
fprintf('Starting continuous 2D tracking at %.0f Hz...\n', 1/Ts);  % Show sampling rate
fprintf('Press the STOP button or Ctrl+C to end tracking.\n\n');   % Explain how to stop

% ===== CONTINUOUS DATA COLLECTION LOOP =====
k = 0;     % Initialize sample counter (tracks how many samples we've collected)
tic;       % Start MATLAB's timer to track elapsed time

% Infinite loop - runs until user stops it
while true
    k = k + 1;     % Increment sample counter each iteration
    pause(Ts);     % Wait for one sampling period (8.33ms) to maintain 120 Hz rate
    t = toc;       % Get elapsed time in seconds since tic was called
    
    % ===== CHECK FOR STOP CONDITIONS =====
    % Check if user pressed the STOP button
    if get(stopButton, 'UserData')  % UserData becomes true when button clicked
        fprintf('\nStop button pressed. Ending tracking...\n');
        break;  % Exit the while loop
    end
    
    % Check if user closed the figure window
    if ~ishandle(fig)  % ishandle returns false if figure no longer exists
        fprintf('\nFigure closed. Ending tracking...\n');
        break;  % Exit the while loop
    end
    
    % ===== READ POSITION DATA FROM MQTT =====
    % Read the latest position message from the MQTT topic
    posTable = read(mqttClient, Topic="rb/limo780/pos");  % Returns table with Data field
    
    % Process the received data
    if ~isempty(posTable)  % Check if we actually received data
        rawData = posTable.Data{end};           % Extract the most recent message (last row)
                                                 % Format example: "[-1.4629, 0.1572, 0.1370]"
        
        rawData = erase(rawData, ["[", "]"]);   % Remove square brackets from string
                                                 % Result: "-1.4629, 0.1572, 0.1370"
        
        numericPos = str2double(split(rawData, ","))';  % Split by comma, convert to numbers
                                                         % split() creates cell array
                                                         % str2double() converts to numbers
                                                         % ' transposes to row vector
    else
        numericPos = [NaN NaN NaN];  % If no data received, use NaN (Not a Number)
                                     % This prevents errors and marks missing data
    end
    
    numericPos = numericPos(:)';  % Ensure it's a row vector [1x3]
                                  % (:) makes column, then ' transposes to row
    
    % Store the data with timestamp
    % dataLog format: each row is [time, X, Z_elevation, Y]
    dataLog = [dataLog; t, numericPos];  % Append new row to bottom of dataLog matrix
    
    % ===== UPDATE VISUALIZATION =====
    % Only update plot if we have valid data (not NaN)
    if ~any(isnan(numericPos))  % any() returns true if ANY element is NaN
                                % ~any() returns true only if NO elements are NaN
        
        % ===== COORDINATE MAPPING =====
        % MQTT sends data as: [X_horizontal, Z_elevation, Y_horizontal]
        % We want 2D plot showing: X (left/right) vs Y (forward/back)
        % So we take positions 1 and 3, skip position 2 (elevation)
        
        plotX = numericPos(1);  % X position (horizontal, left/right) - MQTT index 1
        plotY = numericPos(3);  % Y position (horizontal, forward/back) - MQTT index 3
                                % Note: numericPos(2) is Z elevation, which we ignore for 2D plot
        
        % ===== UPDATE MARKER POSITION =====
        %% set() updates properties of existing graphics object (the scatter plot 'h')
        set(h, 'XData', plotX, ...  % Move marker to new X coordinate
               'YData', plotY);      % Move marker to new Y coordinate
        
      % ===== UPDATE TEXT LABEL =====
        % Update the text showing coordinates to follow the robot
        set(posText, 'Position', [plotX, plotY + 0.1], ...  % Place text slightly above marker
                     'String', sprintf('X:%.2f Y:%.2f', plotX, plotY));  % Format: "X:1.23 Y:4.56"

        % ===== AUTO-ADJUST AXIS LIMITS (EVERY 10 SAMPLES) =====
        % Updating limits every frame causes jitter, so we do it every 10th frame
        if mod(k, 10) == 0  % mod(k,10)==0 is true when k is divisible by 10 (k=10,20,30,...)
            
            % Extract all valid (non-NaN) data points collected so far
            validIdx = ~isnan(dataLog(:,2));  % Find rows where X coordinate (column 2) is valid
                                              % Returns logical array: true for valid rows
            
            validData = dataLog(validIdx, 2:4);  % Extract only valid rows, columns 2-4 [X, Z, Y]
            
            % Only adjust limits if we have at least 2 data points
            if size(validData, 1) > 1  % size(validData,1) gives number of rows
                
                % ===== CALCULATE RANGE FOR EACH AXIS =====
                %% Create [min, max] range for X and Y coordinates
                xRange = [min(validData(:,1)), max(validData(:,1))];  % X is column 1 of validData
                yRange = [min(validData(:,3)), max(validData(:,3))];  % Y is column 3 of validData
                                                                      % (column 2 is Z, which we skip)
                
                % ===== ADD PADDING AROUND DATA =====
                %% We add extra space around the data so the marker isn't at the edge
                padding = 0.2;  % 20% padding as a fraction of data range
                
                % Calculate padding amount for X axis
                xPad = max((xRange(2) - xRange(1)) * padding, 0.5);  
                    % (xRange(2) - xRange(1)) = data span in X direction
                    % * padding = 20% of that span
                    % max(..., 0.5) = use at least 0.5 meters of padding
                
                % Calculate padding amount for Y axis  
                yPad = max((yRange(2) - yRange(1)) * padding, 0.5);  % Same logic as X
                
                % ===== APPLY NEW AXIS LIMITS =====
                % Set axis limits to data range plus padding on both sides
                xlim([xRange(1) - xPad, xRange(2) + xPad]);  % Expand left and right
                ylim([yRange(1) - yPad, yRange(2) + yPad]);  % Expand down and up
            end
        end
        
        % ===== PRINT STATUS TO COMMAND WINDOW (ONCE PER SECOND) =====
        if mod(k, 120) == 0  % k=120,240,360... (every 120 samples = 1 second at 120Hz)
            fprintf('Time: %.1fs | Position: X=%.2f, Y=%.2f | Samples: %d\n', ...
                    t, plotX, plotY, k);  % Show: elapsed time, current position, total samples
        end
    end
    
    % ===== FORCE GRAPHICS UPDATE =====
    drawnow limitrate;  %% Forces MATLAB to update the figure NOW
                        % 'limitrate' limits updates to ~20Hz for efficiency
                        % Without this, plot wouldn't update until loop finishes
end

% ===== POST-PROCESSING AND SUMMARY =====
% This section runs after the loop ends (user pressed STOP)

fprintf('\n=== Tracking Session Summary ===\n');
fprintf('Total samples collected: %d\n', size(dataLog, 1));  % Total rows in dataLog
fprintf('Total duration: %.1f seconds\n', dataLog(end, 1));  % Last timestamp in column 1

% ===== CALCULATE STATISTICS =====
% Extract all valid data for analysis
validIdx = ~isnan(dataLog(:,2));          % Find rows with valid X values
validData = dataLog(validIdx, 2:4);       % Get [X, Z, Y] for valid rows

if ~isempty(validData)  % Only calculate stats if we have valid data
    fprintf('\n--- Room Dimensions Detected ---\n');
    
    % Report X axis range and span
    fprintf('X range: [%.3f, %.3f] m (span: %.3f m)\n', ...
            min(validData(:,1)), ...        % Minimum X value
            max(validData(:,1)), ...        % Maximum X value
            max(validData(:,1)) - min(validData(:,1)));  % Span = max - min
    
    % Report Y axis range and span (remember Y is in column 3)
    fprintf('Y range: [%.3f, %.3f] m (span: %.3f m)\n', ...
            min(validData(:,3)), ...        % Minimum Y value
            max(validData(:,3)), ...        % Maximum Y value
            max(validData(:,3)) - min(validData(:,3)));  % Span = max - min
    
    % Report Z (elevation) for reference, even though we didn't plot it
    fprintf('Z range: [%.3f, %.3f] m (span: %.3f m) [not plotted]\n', ...
            min(validData(:,2)), ...        % Minimum Z value
            max(validData(:,2)), ...        % Maximum Z value  
            max(validData(:,2)) - min(validData(:,2)));  % Span = max - min
    
    % ===== DATA QUALITY METRICS =====
    numValid = size(validData, 1);          % Count of valid samples
    numTotal = size(dataLog, 1);            % Count of total samples (including NaN)
    dataRate = numValid / dataLog(end, 1);  % Valid samples per second
    
    fprintf('\n--- Data Quality ---\n');
    
    % Report percentage of samples that were valid (not missing/NaN)
    fprintf('Valid samples: %d / %d (%.1f%%)\n', ...
            numValid, numTotal, 100*numValid/numTotal);
    
    % Report actual achieved sampling rate (may be less than 120Hz if data was missing)
    fprintf('Effective sample rate: %.1f Hz\n', dataRate);
    
    % Calculate and report average position over entire session
    avgX = mean(validData(:,1));  % Mean of all X values
    avgY = mean(validData(:,3));  % Mean of all Y values
    avgZ = mean(validData(:,2));  % Mean of all Z values
    fprintf('Average position: X=%.3f, Y=%.3f, Z=%.3f m\n', avgX, avgY, avgZ);
end

% ===== OPTIONAL: SAVE DATA TO FILE =====
saveData = input('\nSave data to file? (y/n): ', 's');  % Ask user if they want to save
                                                         % 's' means read input as string

if strcmpi(saveData, 'y')  % strcmpi = case-insensitive string comparison
    
    % Create filename with timestamp: mocap_data_20241114_153045.mat
    filename = sprintf('mocap_data_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
    
    % Save dataLog and sampling period to .mat file
    save(filename, 'dataLog', 'Ts');
    
    fprintf('Data saved to %s\n', filename);
    fprintf('Note: dataLog format is [time, X, Z_elevation, Y]\n');
end

%
% ===== OPTIONAL: POST-ANALYSIS VISUALIZATION =====
% Uncomment this section to see the full 2D trajectory path after collection

% figure('Name', 'Full 2D Trajectory');      % Create new figure
% validIdx = ~isnan(dataLog(:,2));            % Find valid data points
% 
% Create 2D scatter plot colored by time
% scatter(dataLog(validIdx,2), ...            % X from column 2
%         dataLog(validIdx,4), ...            % Y from column 4
%         50, ...                              % Marker size
%         dataLog(validIdx,1), ...            % Color by time (column 1)
%         'filled');                          % Filled markers
% 
% grid on;                                    % Show grid
% xlabel('X Position (m)');                   % Label X axis
% ylabel('Y Position (m)');                   % Label Y axis
% title('Complete Robot Trajectory (colored by time)');  % Title
% colorbar;                                   % Show color scale for time
% colormap('jet');                            % Use jet colormap (blue→red)
% axis equal;                                 % Equal scaling%