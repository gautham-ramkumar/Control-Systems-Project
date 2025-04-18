function robot_path_planning_and_control
    % Main Script for Robot Path Planning and Control
    % This script implements robot path planning using Probabilistic Roadmap (PRM)
    % and path following using SMC controller
    
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
    fileName = ['SMC_Metrics_' mazeName '.mat'];
    videoName = ['SMC_Simulation_' mazeName '.mp4'];

    map = occupancyMap(maze.binaryMap, 1); % Generate Occupancy Map from binary grid

    startLocation = maze.startLocation;
    goalLocation = maze.goalLocation;
    numNodes = maze.numNodes;

    fprintf('Maze %d: %s\n', maze_number, mazeName);
    fprintf('Start Location: [%d, %d]\n', startLocation);
    fprintf('Goal Location:  [%d, %d]\n', goalLocation);
    fprintf('Num Nodes: %d\n\n', numNodes);
    fprintf('Using SMC Controller\n\n');

    % Path Planning
    path = pathPlanningPRM(map, startLocation, goalLocation, numNodes);

    if isempty(path)
        disp("Simulation aborted: No valid path found.");
        return;
    end

    disp_path(map, path, startLocation, goalLocation);

    % Run SMC controller
    [trajectory, velocities, omegas, totalTime, numControlSteps] = simulateRobotSMCPathFollowing(map, path, startLocation, goalLocation, saveVideo, limits, videoName);
    
    % Calculate metrics
    meanErr = computeMeanTrackingError(trajectory, path);
    fprintf('Mean Tracking Error: %.2f meters\n', meanErr);
    
    % Save data
    if saveData
        saveSimulationMetrics(fileName, mazeName, size(maze.binaryMap), meanErr, numControlSteps, numNodes, omegas, path, totalTime, trajectory, velocities);
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

% SMC Controller
function [v, omega] = computeSMCOutput(robotPose, refPose, v_ref, omega_ref)
    % Position errors in the reference frame
    dx = robotPose(1) - refPose(1);
    dy = robotPose(2) - refPose(2);
    theta_ref = refPose(3);
    
    % Transform errors to reference frame
    ex = dx * cos(theta_ref) + dy * sin(theta_ref);
    ey = -dx * sin(theta_ref) + dy * cos(theta_ref);
    etheta = wrapToPi(robotPose(3) - theta_ref);

    % Modified sliding surface with decreased heading error weight for smoother tracking
    s = -ey - 0.7 * etheta;  % Reduced from 1.0 to 0.7
    
    % Reduced control parameters for less aggressive behavior in complex mazes
    eta = 2.5;  % Reduced from 3.0
    K = 3.0;    % Reduced from 3.5
    phi = 10;    % Reduced from 12
    
    % Smoother control law
    control_signal = eta * tanh(phi * s) + K * s;
    
    % Less aggressive velocity reduction on curves
    alpha = 0.15;  % Reduced from 0.2
    v = max(1.5, min(v_ref / (1 + alpha * abs(ey)), 6.0));
    
    % Wider angular velocity constraints
    omega = omega_ref + control_signal;
    omega = min(max(omega, -2.0), 2.0);  % Increased from ±1.5 to ±2.0
end

% Simulation with SMC controller
function [trajectory, velocities, omegas, totalTime, numControlSteps] = simulateRobotSMCPathFollowing(map, path, startLocation, goalLocation, saveVideo, limits, videoName)
    % Set simulation parameters
    sampleTime = 0.01;
    goalRadius = 2.0;

    % Initialize simulation variables
    simStartTime = tic;
    velocities = [];
    omegas = [];
    numControlSteps = 0;
    
    % Initial pose at start of path
    direction = path(2,:) - path(1,:);
    initial_theta = atan2(direction(2), direction(1));
    robotPose = [path(1,:), initial_theta];
    trajectory = robotPose(1:2);

    % SMC Parameters
    v_ref = 5.0;
    omega_ref = 0.0;

    % Visualization setup
    fig = figure('Name', 'Robot SMC Simulation');
    set(fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);   % Fullscreen
    set(fig, 'MenuBar', 'none', 'ToolBar', 'none');                % Clean UI
    show(map); hold on;
    plot(path(:,1), path(:,2), 'k--', 'LineWidth', 2);
    plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goalLocation(1), goalLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    robotPlot = plot(robotPose(1), robotPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    trajPlot = plot(robotPose(1), robotPose(2), 'b-', 'LineWidth', 1.5);
    headingArrow = quiver(robotPose(1), robotPose(2), cos(robotPose(3)), sin(robotPose(3)), 3, 'b', 'LineWidth', 2);
    title('Robot Path Following with SMC');
    xlabel('X [meters]'); ylabel('Y [meters]');

    % Video recording setup
    if saveVideo
        vWriter = VideoWriter(videoName, 'MPEG-4');
        vWriter.FrameRate = round(1/sampleTime);
        open(vWriter);
    end

    % Simulation Loop
    distanceToGoal = norm(robotPose(1:2) - goalLocation);
    pathIndex = 2;

    while distanceToGoal > goalRadius && pathIndex <= size(path,1)
        numControlSteps = numControlSteps + 1;
        
        % Reference pose (current segment target)
        refPose = [path(pathIndex,:), atan2(path(pathIndex,2) - path(pathIndex-1,2), path(pathIndex,1) - path(pathIndex-1,1))];

        % Compute control using SMC
        [v, omega] = computeSMCOutput(robotPose, refPose, v_ref, omega_ref);

        % Logging velocities and omegas
        velocities = [velocities, v];
        omegas = [omegas, omega];

        % Update robot pose using odometry
        robotPose = updateOdometry(robotPose, v, omega, sampleTime);
        trajectory = [trajectory; robotPose(1:2)];

        % Update visualization
        set(robotPlot, 'XData', robotPose(1), 'YData', robotPose(2));
        set(trajPlot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        set(headingArrow, 'XData', robotPose(1), 'YData', robotPose(2), ...
                        'UData', cos(robotPose(3)), 'VData', sin(robotPose(3)));
        drawnow;
        
        % Record video frame if enabled
        if saveVideo
            frame = getframe(gcf);
            writeVideo(vWriter, frame);
        end
        
        pause(sampleTime); % Reduced pause time for faster simulation

        % Switch to next waypoint if close enough
        if norm(robotPose(1:2) - refPose(1:2)) < 1.0 && pathIndex < size(path,1)
            pathIndex = pathIndex + 1;
        end

        distanceToGoal = norm(robotPose(1:2) - goalLocation);
    end

    % Mark final position
    plot(robotPose(1), robotPose(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    legend('Planned Path', 'Start', 'Goal', 'Robot', 'Trajectory', 'Final Pose');
    hold off;

    totalTime = toc(simStartTime);
    
    % Close video writer
    if saveVideo
        close(vWriter);
        fprintf('Simulation video saved to: %s\n', videoName);
    end

    disp('Robot has reached the goal using SMC!');
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

% Save Simulation Metrics Function - Extended version
function saveSimulationMetrics(fileName, image_name, mapSize, meanErr, numControlSteps, numNodes, omegas, path, totalTime, trajectory, velocities)
    % Organize everything into a struct
    metrics = struct();
    metrics.fileName = fileName;
    metrics.image_name = image_name;
    metrics.mapSize = mapSize;
    metrics.meanError = meanErr;
    metrics.numControlSteps = numControlSteps;
    metrics.numNodes = numNodes;
    metrics.omegas = omegas;
    metrics.path = path;
    metrics.totalTime = totalTime;
    metrics.trajectory = trajectory;
    metrics.velocities = velocities;

    % Save to .mat file
    save(fileName, '-struct', 'metrics');
    fprintf('Saved simulation metrics to: %s\n', fileName);
end