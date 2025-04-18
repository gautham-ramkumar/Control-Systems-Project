function robot_path_planning_and_control
    % Main Script for Robot Path Planning and Control
    % This script implements robot path planning using Probabilistic Roadmap (PRM)
    % and path following using PID controller
    
    clc
    clear 
    close all

    % Loading Maze Data 
    data = load('mazeLibrary.mat');
    mazeLibrary = data.mazeLibrary; % Loading Maze Data 
    limits = data.limits; % Loading Limits

    % Create maze selection UI
    maze_options = {
        '1 - 15x15 Maze', 
        '2 - 20x20 Maze', 
        '3 - 25x25 Maze', 
        '4 - 30x30 Maze', 
        '5 - 35x35 Maze', 
        '6 - 40x40 Maze', 
        '7 - 45x45 Maze', 
        '8 - 50x50 Maze'
    };

    [maze_idx, maze_ok] = listdlg('PromptString', 'Select Maze Size:', ...
                               'SelectionMode', 'single', ...
                               'ListString', maze_options, ...
                               'Name', 'Maze Selection');

    if ~maze_ok
        disp('Maze selection cancelled. Exiting.');
        return;
    end

    maze_number = maze_idx; % Maze number is the same as the index

    % Set video and data saving flags
    saveVideo = true; % Always save videos
    saveData = true;  % Always save data

    maze = mazeLibrary{maze_number};
    mazeName = maze.name;

    % Set up file names
    fileName = ['PID_Metrics_' mazeName '.mat'];
    videoName = ['PID_Simulation_' mazeName '.mp4'];

    map = occupancyMap(maze.binaryMap, 1); % Generate Occupancy Map from binary grid

    startLocation = maze.startLocation;
    goalLocation = maze.goalLocation;
    numNodes = maze.numNodes;

    fprintf('Maze %d: %s\n', maze_number, mazeName);
    fprintf('Start Location: [%d, %d]\n', startLocation);
    fprintf('Goal Location:  [%d, %d]\n', goalLocation);
    fprintf('Num Nodes: %d\n\n', numNodes);
    fprintf('Using PID Controller\n\n');

    % Path Planning
    path = pathPlanningPRM(map, startLocation, goalLocation, numNodes);

    if isempty(path)
        disp("Simulation aborted: No valid path found.");
        return;
    end

    disp_path(map, path, startLocation, goalLocation);

    % Run PID Controller
    [trajectory, velocities, omegas, totalTime, numControlSteps] = simulateRobotPID(map, path, startLocation, goalLocation, saveVideo, limits, videoName);
    
    % Calculate metrics
    meanErr = computeMeanTrackingError(trajectory, path);
    fprintf('Mean Tracking Error: %.2f meters\n', meanErr);
    
    % Save data
    if saveData
        saveSimulationMetricsBasic(fileName, meanErr, numControlSteps, omegas, path, totalTime, trajectory, velocities);
    end

    disp('Simulation Ended!!');
end

% Path Planning Function using PRM
function path = pathPlanningPRM(map, startLocation, goalLocation, numNodes)
    % Start timing the PRM algorithm execution
    tic;
    
    inflatedMap = copy(map);
    inflationRadius = 4;
    inflate(inflatedMap, inflationRadius);

    % Create PRM object
    prm = mobileRobotPRM(inflatedMap, numNodes);
    disp('Finding Path...');  
    path = findpath(prm, startLocation, goalLocation);

    planningTime = toc;

    if isempty(path)
        disp('No valid path found between start and goal points.');
        return;
    end

    disp('Path Found!!');
    pathLength = sum(vecnorm(diff(path)'));
    disp(['Path Length: ', num2str(pathLength)]);
    disp(['Planning Time: ', num2str(planningTime), ' seconds']);
end

% Display Path Function
function disp_path(map, path, startLocation, goalLocation)
    if isempty(path)
        disp('No valid path found, cannot display.');
        return;
    end
    figure;
    show(map);
    hold on;
    plot(path(:, 1), path(:, 2), 'r-', 'LineWidth', 2); % Red trajectory
    plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Green Start
    plot(goalLocation(1), goalLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Red Goal
    title('Planned Path');
    xlabel('X [meters]');
    ylabel('Y [meters]');
    legend('Trajectory', 'Start Position', 'Goal Position');
    hold off;
end

% Update Robot Pose based on control
function nextPose = updateOdometry(pose, v, omega, dt)
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    
    x_next = x + v * cos(theta) * dt;
    y_next = y + v * sin(theta) * dt;
    theta_next = wrapToPi(theta + omega * dt); % Wrap theta to [-pi, pi]
    
    nextPose = [x_next, y_next, theta_next];
end

% PID Controller Function
function [v, omega] = PIDController(robotPose, refPose, v_ref, omega_ref, Kp, Ki, Kd, dt, prevError, integral, limits)
    error = refPose(1:2) - robotPose(1:2);  % Position error (x, y)
    
    % Calculate angle to goal
    angleToGoal = atan2(error(2), error(1));
    headingError = wrapToPi(angleToGoal - robotPose(3));

    % Calculate distance error
    distError = norm(error);
    
    % Calculate linear velocity - now using v_ref as a base 
    v = v_ref * (1.0 - 0.8 * exp(-distError)); % Approaches v_ref as distance increases
    
    % Calculate angular velocity based on PID
    omega = omega_ref + Kp(2) * headingError + Ki(2) * integral + Kd(2) * (headingError - prevError) / dt;

    % Update integral term
    integral = integral + headingError * dt;
    
    % Anti-windup for integral term
    integral = max(min(integral, 2.0), -2.0);
    
    % Apply velocity limits
    v = max(min(v, limits.v_max), limits.v_min);
    omega = max(min(omega, limits.omega_max), limits.omega_min);

    % Return updated error for next time step
    prevError = headingError;
end

% Simulation Function using PID Control
function [trajectory, velocities, omegas, totalTime, numControlSteps] = simulateRobotPID(map, path, startLocation, goalLocation, saveVideo, limits, videoName)
    sampleTime = 0.01;
    goalRadius = 2.0;
    simStartTime = tic;
    velocities = [];
    omegas = [];
    numControlSteps = 0;

    direction = path(2,:) - path(1,:);
    initial_theta = atan2(direction(2), direction(1));
    robotPose = [path(1,:), initial_theta];
    trajectory = robotPose(1:2);

    % PID Parameters
    Kp = [1.0 1.0];   % Proportional gain for x, y
    Ki = [0.01 0.01]; % Integral gain for x, y
    Kd = [0.1 0.1];   % Derivative gain for x, y
    integral = 0;     % Initialize integral term
    prevError = 0;    % Initialize previous error
    
    % Reference velocities
    v_ref = 5.0;      % Reference linear velocity
    omega_ref = 0.0;  % Reference angular velocity
    
    % Visualization setup
    fig = figure('Name', 'Robot PID Simulation');
    set(fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);   % Fullscreen
    set(fig, 'MenuBar', 'none', 'ToolBar', 'none');                % Clean UI
    show(map); hold on;
    plot(path(:,1), path(:,2), 'k--', 'LineWidth', 2);
    plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goalLocation(1), goalLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    robotPlot = plot(robotPose(1), robotPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    trajPlot = plot(robotPose(1), robotPose(2), 'b-', 'LineWidth', 1.5);
    headingArrow = quiver(robotPose(1), robotPose(2), cos(robotPose(3)), sin(robotPose(3)), 3, 'b', 'LineWidth', 2);
    title('Robot Path Following with PID');
    xlabel('X [meters]'); ylabel('Y [meters]');
    
    if saveVideo
        vWriter = VideoWriter(videoName, 'MPEG-4');
        vWriter.FrameRate = round(1/sampleTime);
        open(vWriter);
    end
    
    distanceToGoal = norm(robotPose(1:2) - goalLocation);
    pathIndex = 2;

    while distanceToGoal > goalRadius && pathIndex <= size(path,1)
        numControlSteps = numControlSteps + 1;
        refPose = [path(pathIndex,:), atan2(path(pathIndex,2) - path(pathIndex-1,2), path(pathIndex,1) - path(pathIndex-1,1))];

        [v, omega] = PIDController(robotPose, refPose, v_ref, omega_ref, Kp, Ki, Kd, sampleTime, prevError, integral, limits);

        velocities = [velocities, v];
        omegas = [omegas, omega];

        robotPose = updateOdometry(robotPose, v, omega, sampleTime);
        trajectory = [trajectory; robotPose(1:2)];

        % Update visualization
        set(robotPlot, 'XData', robotPose(1), 'YData', robotPose(2));
        set(trajPlot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        set(headingArrow, 'XData', robotPose(1), 'YData', robotPose(2), 'UData', cos(robotPose(3)), 'VData', sin(robotPose(3)));
        drawnow;
        
        if saveVideo
            frame = getframe(gcf);
            writeVideo(vWriter, frame);
        end
        
        pause(sampleTime);

        if norm(robotPose(1:2) - refPose(1:2)) < 1.0 && pathIndex < size(path,1)
            pathIndex = pathIndex + 1;
        end

        distanceToGoal = norm(robotPose(1:2) - goalLocation);
    end

    plot(robotPose(1), robotPose(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    legend('Planned Path', 'Start', 'Goal', 'Robot', 'Trajectory', 'Final Position');
    hold off;
    
    totalTime = toc(simStartTime);
    
    % Close video writer
    if saveVideo
        close(vWriter);
        fprintf('Simulation video saved to: %s\n', videoName);
    end
    
    disp('Robot has reached the goal using PID!');
end

% Compute Mean Tracking Error Function
function meanErr = computeMeanTrackingError(trajectory, path)
    % Compute mean Euclidean distance from each actual position to the closest point on the path
    errors = zeros(size(trajectory,1),1);

    for i = 1:length(errors)
        currentPos = trajectory(i,:);
        distances = vecnorm(path - currentPos, 2, 2);  % Euclidean distance to all path points
        errors(i) = min(distances);  % Closest point error
    end

    meanErr = mean(errors);
end

% Save Simulation Metrics Function - Basic version
function saveSimulationMetricsBasic(fileName, meanErr, numControlSteps, omegas, path, totalTime, trajectory, velocities)
    % Organize everything into a struct
    metrics = struct();
    metrics.fileName = fileName;
    metrics.meanError = meanErr;
    metrics.numControlSteps = numControlSteps;
    metrics.omegas = omegas;
    metrics.path = path;
    metrics.totalTime = totalTime;
    metrics.trajectory = trajectory;
    metrics.velocities = velocities;

    % Save to .mat file
    save(fileName, '-struct', 'metrics');

    fprintf('Saved simulation metrics to: %s\n', fileName);
end