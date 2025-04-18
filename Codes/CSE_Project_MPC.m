
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


function mpcParams = initializeMPCController(sampleTime, limits)
    % Function to initialize MPC controller for robot path tracking
    % Returns a structure with MPC parameters
    
    % Create a parameter structure for MPC
    mpcParams = struct();
    mpcParams.predictionHorizon = 15;  % Prediction horizon
    mpcParams.controlHorizon = 5;      % Control horizon
    mpcParams.sampleTime = sampleTime; % Sampling time
    
    % Control weights tuned for good performance
    mpcParams.Q = diag([30, 30, 8]);   % Higher position tracking weights
    mpcParams.R = diag([0.7, 1.2]);    % Control effort penalty
    
    % Use the provided velocity limits
    mpcParams.v_min = limits.v_min;
    mpcParams.v_max = limits.v_max;
    mpcParams.omega_min = limits.omega_min;
    mpcParams.omega_max = limits.omega_max;
    
    % Path tracking parameters
    mpcParams.pathErrorWeight = 10.0;  % Path tracking error weight
    mpcParams.waypointThreshold = 0.8; % Waypoint switching threshold
    
    % Stuck detection parameters
    mpcParams.stuckCounter = 0;
    mpcParams.stuckThreshold = 0.05;
    mpcParams.stuckTimeout = 20;
end

function [v, omega] = computeMPCOutput(robotPose, refPose, v_ref, omega_ref,  limits, path, pathIndex)
    % Function to compute MPC control outputs
    % Uses same parameter structure as LQR for consistency
    
    % Create MPC parameters on-the-fly
    sampleTime = 0.01; % Same sample time as in simulation
    mpcParams = initializeMPCController(sampleTime, limits);
    %mpcParams.Q = Q;   % Use the provided Q matrix
    %mpcParams.R = R;   % Use the provided R matrix
    
    % Look ahead in the path to identify turns
    lookAheadIndex = min(pathIndex + 3, size(path, 1));
    if pathIndex < lookAheadIndex && lookAheadIndex <= size(path, 1)
        % Calculate turn angle
        currentDir = path(pathIndex,:) - path(max(pathIndex-1, 1),:);
        aheadDir = path(lookAheadIndex,:) - path(pathIndex,:);
        
        if norm(currentDir) > 0 && norm(aheadDir) > 0
            currentDir = currentDir / norm(currentDir);
            aheadDir = aheadDir / norm(aheadDir);
            turnAngle = acos(min(1, max(-1, dot(currentDir, aheadDir))));
            
            % Adjust velocity for sharp turns
            if turnAngle > 0.7  % About 40 degrees
                v_ref = v_ref * 0.9;  % Reduce speed for sharp turns
            end
        end
    end
    
    % Set up optimization variables
    Nu = mpcParams.controlHorizon;
    
    % Initial control sequence
    u0 = zeros(2*Nu, 1);
    for i = 1:Nu
        u0(2*i-1) = v_ref;
        u0(2*i) = omega_ref;
    end
    
    % Define bounds for control inputs
    lb = zeros(2*Nu, 1);
    ub = zeros(2*Nu, 1);
    for i = 1:Nu
        lb(2*i-1) = limits.v_min;
        lb(2*i) = limits.omega_min;
        ub(2*i-1) = limits.v_max;
        ub(2*i) = limits.omega_max;
    end
    
    % Optimization options
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
                          'MaxIterations', 100, 'MaxFunctionEvaluations', 1000);
    
    % Try optimization
    try
        [u_opt, ~] = fmincon(@(u) objectiveFunction(u, mpcParams, robotPose, refPose, path), ...
                         u0, [], [], [], [], lb, ub, [], options);
    catch
        % Fallback to initial control if optimization fails
        warning('MPC optimization failed. Using fallback control.');
        u_opt = u0;
    end
    
    % Extract first control action (MPC principle)
    v = u_opt(1);
    omega = u_opt(2);
    
    % Apply control limits
    v = min(max(v, limits.v_min), limits.v_max);
    omega = min(max(omega, limits.omega_min), limits.omega_max);
end

function J = objectiveFunction(u, mpcParams, x0, xref, path)
    % Objective function for MPC optimization
    
    % Parameters
    N = mpcParams.predictionHorizon;
    Nu = mpcParams.controlHorizon;
    dt = mpcParams.sampleTime;
    Q = mpcParams.Q;
    R = mpcParams.R;
    
    % Initialize cost
    J = 0;
    
    % Ensure column vectors
    x0 = x0(:);
    xref = xref(:);
    
    % Reshape control sequence
    U = reshape(u, 2, Nu);
    
    % Simulate system forward from current state
    x = x0;
    
    for k = 1:N
        % Get control for this step
        if k <= Nu
            v = U(1,k);
            omega = U(2,k);
        else
            % Hold last control for remaining horizon
            v = U(1,Nu);
            omega = U(2,Nu);
        end
        
        % Update state using kinematics model
        x_next = updateOdometry(x, v, omega, dt);
        x_next = x_next(:);
        
        % State error cost
        e = x_next - xref;
        e(3) = wrapToPi(e(3));  % Handle angle wrapping for theta
        
        % Time-weighted cost (higher weight for near-term predictions)
        time_weight = 1.0 + 0.1 * k;
        
        % Calculate state cost manually to avoid dimension issues
        state_cost = 0;
        for i = 1:3
            for j = 1:3
                state_cost = state_cost + e(i) * Q(i,j) * e(j) * time_weight;
            end
        end
        J = J + state_cost;
        
        % Add path tracking cost
        % Find closest point on path
        distances = vecnorm(path - x_next(1:2)', 2, 2);
        pathError = min(distances);
        J = J + mpcParams.pathErrorWeight * pathError * time_weight;
        
        % Control effort cost
        J = J + v*R(1,1)*v + omega*R(2,2)*omega;
        
        % Control rate cost (smoother control changes)
        if k > 1
            if k <= Nu
                prev_v = U(1,k-1);
                prev_omega = U(2,k-1);
            else
                prev_v = U(1,Nu);
                prev_omega = U(2,Nu);
            end
            
            % Penalize rapid changes
            J = J + 0.3*(v - prev_v)^2 + 2.0*(omega - prev_omega)^2;
        end
        
        % Update state for next iteration
        x = x_next;
    end
end


function [trajectory,velocities,omegas,totalTime,numControlSteps] = simulateRobotMPCPathFollowing(map, path, startLocation, goalLocation,saveVideo,limits, videoName)
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
    
    
    v_ref = 5.0;
    omega_ref = 0.0;
    mpcParams = initializeMPCController(sampleTime, limits);
    fig = figure;
    set(gcf,'Position',[100,100,800,800]);



    % Visualization setup
    fig = figure('Name', 'Robot MPC Simulation');
    set(fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);   % Fullscreen
    set(fig, 'MenuBar', 'none', 'ToolBar', 'none');                % Clean UI
    show(map); hold on;
    plot(path(:,1), path(:,2), 'k--', 'LineWidth', 2);
    plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goalLocation(1), goalLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    robotPlot = plot(robotPose(1), robotPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    trajPlot = plot(robotPose(1), robotPose(2), 'b-', 'LineWidth', 1.5);
    headingArrow = quiver(robotPose(1), robotPose(2), cos(robotPose(3)), sin(robotPose(3)), 3, 'b', 'LineWidth', 2);
    title('Robot Path Following with MPC');
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
        [v, omega] = computeMPCOutput(robotPose, refPose, v_ref, omega_ref,limits, path, pathIndex);
       
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

maze_number = 8 ; % Change This Value (1 - 8) total 8 mazes in the data file 

maze = mazeLibrary{maze_number};

mazeName = maze.name ;

fileName = ['MPC_Metrices_' mazeName '.mat']; % File name to save the simulation output data for analysis
videoName = ['MPC_Simulation_' mazeName '.mp4'];  % Video file name to save the maze simulation video
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

[trajectory, velocities, omegas,totalTime,numControlSteps] = simulateRobotMPCPathFollowing(map, path, startLocation, goalLocation, saveVideo, limits, videoName);

meanErr = computeMeanTrackingError(trajectory, path);

fprintf('Mean Tracking Error: %.2f meters\n', meanErr);

if saveData
    saveSimulationMetrics(fileName, meanErr, numControlSteps, omegas, path, totalTime, trajectory, velocities);
end

disp('Simulation Ended !!');