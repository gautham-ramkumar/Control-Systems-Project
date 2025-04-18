
function path = pathPlanningPRM(map, startLocation, goalLocation, numNodes)
    % Start timing the PRM algorithm execution
    tic;

    inflatedMap = copy(map);

    inflationRadius = 4;

    inflate(inflatedMap,inflationRadius);

    % Create a PRM object with the specified number of nodes
    prm = mobileRobotPRM(inflatedMap, numNodes);

    disp('Finding Path...');
    % Find the path between start and goal points
    path = findpath(prm, startLocation, goalLocation);

    % Calculate the time taken for planning
    planningTime = toc;

    % Check if a path was found
    if isempty(path)
        disp('No valid path found between the start and goal points.');
        return;
    end
    
    disp('Path Found!!');
    % Calculate the total path length by summing Euclidean distances between waypoints
    pathLength = sum(vecnorm(diff(path)'));
    
    % Display path length and planning time in the command window
    disp(['Path Length: ', num2str(pathLength)]);
    disp(['Planning Time: ', num2str(planningTime), ' seconds']);

end

function disp_path(map, path, startLocation, goalLocation)
     
    if isempty(path)
            disp('No valid path found, cannot display.');
            return;
    end
    figure;
    show(map);
    hold on;

    % Plot the planned path
    plot(path(:, 1), path(:, 2), 'r-', 'LineWidth', 2); % Red trajectory
    plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Green Start
    plot(goalLocation(1), goalLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Red Goal

    % Add labels
    title('Planned PRM Path on Original Map');
    xlabel('X [meters]');
    ylabel('Y [meters]');
    legend('Trajectory', 'Start Position', 'Goal Position');
    
    hold off;
end

function nextPose = updateOdometry(pose, v, omega, dt)
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    x_next = x + v * cos(theta) * dt;
    y_next = y + v * sin(theta) * dt;
    theta_next = wrapToPi(theta + omega * dt);

    nextPose = [x_next, y_next, theta_next];
end

function [v, omega] = computeLQROutput(robotPose, refPose, v_ref, omega_ref, Q, R, limits)
    
    theta_ref = refPose(3);
    
    % Linearize around the reference pose
    A = [0, 0, -v_ref * sin(theta_ref);
         0, 0,  v_ref * cos(theta_ref);
         0, 0, 0];

    B = [cos(theta_ref), 0;
         sin(theta_ref), 0;
         0,              1];

    % Calculate LQR gain
    K = lqr(A, B, Q, R);

    % Compute error between current pose and reference
    e = [robotPose(1) - refPose(1);
         robotPose(2) - refPose(2);
         wrapToPi(robotPose(3) - refPose(3))];

    % Compute control input
    u = -K * e + [v_ref; omega_ref];
    v = u(1);
    omega = u(2);
    
    v = min(max(v, limits.v_min), limits.v_max);
    omega = min(max(omega, limits.omega_min), limits.omega_max);

end

function [trajectory,velocities,omegas,totalTime,numControlSteps] = simulateRobotLQRPathFollowing(map, path, startLocation, goalLocation,saveVideo,limits, videoName)
    sampleTime = 0.01;
    goalRadius = 2.0;

    simStartTime= tic;
    velocities=[];
    omegas=[];
    numControlSteps = 0;


    % Initial pose at start of path
    direction = path(2,:) - path(1,:);
    initial_theta = atan2(direction(2), direction(1));
    robotPose = [path(1,:), initial_theta];
    trajectory = robotPose(1:2);

    % LQR Parameters
    Q = diag([15, 15,5]); % Tracking error penalty
    R = diag([1, 1]);      % Control effort penalty
    v_ref = 5.0;
    omega_ref = 0.0;
    
    fig = figure;
    set(gcf,'Position',[100,100,800,800]);



    % Visualization setup
    fig = figure('Name', 'Robot LQR Simulation');
    set(fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);   % Fullscreen
    set(fig, 'MenuBar', 'none', 'ToolBar', 'none');                % Clean UI
    show(map); hold on;
    plot(path(:,1), path(:,2), 'k--', 'LineWidth', 2);
    plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goalLocation(1), goalLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    robotPlot = plot(robotPose(1), robotPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    trajPlot = plot(robotPose(1), robotPose(2), 'b-', 'LineWidth', 1.5);
    headingArrow = quiver(robotPose(1), robotPose(2), cos(robotPose(3)), sin(robotPose(3)), 3, 'b', 'LineWidth', 2);
    title('Robot Path Following with LQR');
    xlabel('X [meters]'); ylabel('Y [meters]');
    
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

        % Compute control using LQR
        [v, omega] = computeLQROutput(robotPose, refPose, v_ref, omega_ref, Q, R, limits);
       
        %logging velocities and omegas
        velocities = [velocities,v];
        omegas = [omegas,omega];

        % Update robot pose using odometry
        robotPose = updateOdometry(robotPose, v, omega, sampleTime);
        trajectory = [trajectory; robotPose(1:2)];

        % Update visualization
        set(robotPlot, 'XData', robotPose(1), 'YData', robotPose(2));
        set(trajPlot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        set(headingArrow, 'XData', robotPose(1), 'YData', robotPose(2), ...
                          'UData', cos(robotPose(3)), 'VData', sin(robotPose(3)));
        drawnow;
        if saveVideo
            frame = getframe(gcf);
            writeVideo(vWriter, frame);
        end
        pause(sampleTime);

        % Switch to next waypoint if close enough
        if norm(robotPose(1:2) - refPose(1:2)) < 1.0 && pathIndex < size(path,1)
            pathIndex = pathIndex + 1;
        end

        distanceToGoal = norm(robotPose(1:2) - goalLocation);
    end
    
    plot(robotPose(1), robotPose(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    legend('Planned Path', 'Start', 'Goal', 'Robot', 'Trajectory', 'Final Pose');
    hold off;
    
    totalTime = toc(simStartTime);
    % Close video writer
    if saveVideo
        close(vWriter);
        fprintf('Simulation video saved to: %s\n', videoName);
    end

    disp('Robot has reached the goal using LQR!');
end

function meanError = computeMeanTrackingError(trajectory, path)

    % Compute mean Euclidean distance from each actual position to the closest point on the path
    errors = zeros(size(trajectory,1),1);

    for i = 1:length(errors)
        currentPos = trajectory(i,:);
        distances = vecnorm(path - currentPos, 2, 2);  % Euclidean distance to all path points
        errors(i) = min(distances);  % Closest point error
    end

    meanError = mean(errors);
end

function saveSimulationMetrics(fileName, meanErr, numControlSteps, omegas, path, totalTime, trajectory, velocities)

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

    fprintf('✅ Saved simulation metrics to: %s\n', fileName);
end
%% Main Code 
clc
clear 
close all

% Loading Maze Data 
data = load('mazeLibrary.mat');

mazeLibrary = data.mazeLibrary; % Loading Maze Data 

limits = data.limits;% Loading Limits

maze_number = 2 ; % Change This Value (1 - 8) total 8 mazes in the data file 

maze = mazeLibrary{maze_number};

mazeName = maze.name ;

fileName = ['LQR_Metrices_' mazeName '.mat']; % File name to save the simulation output data for analysis
videoName = ['LQR_Simulation_' mazeName '.mp4'];  % Video file name to save the maze simulation video
saveVideo = true;
saveData = true;

map = occupancyMap(maze.binaryMap, 1);% Generate Occupancy Map from binary grid

startLocation = maze.startLocation;
goalLocation = maze.goalLocation;
numNodes = maze.numNodes;

fprintf('Maze %d: %s\n', maze_number, mazeName);
fprintf('Start Location: [%d, %d]\n', startLocation);
fprintf('Goal Location:  [%d, %d]\n', goalLocation);
fprintf('Num Nodes: %d\n\n', numNodes);

path = pathPlanningPRM(map, startLocation, goalLocation, numNodes);% Path Planning Function

if isempty(path)
    disp("Simulation aborted: No valid path found.");
    return;
end

disp_path(map,path,startLocation,goalLocation);

[trajectory, velocities, omegas,totalTime,numControlSteps] = simulateRobotLQRPathFollowing(map, path, startLocation, goalLocation, saveVideo, limits, videoName);

meanErr = computeMeanTrackingError(trajectory, path);

fprintf('Mean Tracking Error: %.2f meters\n', meanErr);

if saveData
    saveSimulationMetrics(fileName, meanErr, numControlSteps, omegas, path, totalTime, trajectory, velocities);
end

disp('Simulation Ended !!');