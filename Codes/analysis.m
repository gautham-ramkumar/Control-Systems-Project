%% Main Data Folder
dataRoot = 'F:\MS Robotics\ME5659 Control Systems Engineering\Final Project\Final Data\Mat Files';

% List of controller names
controllerNames = ["LQR", "MPC", "SMC","PID"];  % Now including 3 controllers

% List of maze sizes
mazeSizes = ["15x15", "20x20", "25x25", "30x30", "35x35", "40x40", "45x45"];

% Initialize results struct
results = struct();

missingFiles = {};  % Initialize cell array to store names of missing files

for c = 1:length(controllerNames)
    controller = controllerNames(c);
    
    controllerFolder = fullfile(dataRoot, controller);

    for i = 1:length(mazeSizes)
        % Compose file name
        fileName = sprintf('%s_Metrices_%s.mat', controller, mazeSizes(i));
        % Compose file path 
        fullPath = fullfile(controllerFolder, fileName);

        if isfile(fullPath)
            data = load(fullPath);

            % Use dynamic field names
            results.(controller)(i).meanError = data.meanError;
            results.(controller)(i).totalTime = data.totalTime;
            results.(controller)(i).numControlSteps = data.numControlSteps;
            results.(controller)(i).trajectory = data.trajectory;
            results.(controller)(i).velocities = data.velocities;
            results.(controller)(i).omegas = data.omegas;
            results.(controller)(i).path = data.path;
        else
            % Add missing file name (only the file name) to the list, ensuring .mat extension is attached properly
            missingFiles{end+1} = sprintf('%s.mat', strrep(fileName, '.mat', ''));  % Directly use fileName
        end
    end
end

% If there are missing files, throw an error and stop execution
if ~isempty(missingFiles)
    error('The following files are missing:\n%s', strjoin(missingFiles, '\n'));  % Throw error with list of missing files
else
    disp('All files are present.');
end

% Plot results for both old and new combined plots
plotCombinedTotalTime(results, controllerNames, mazeSizes);
plotCombinedMeanError(results, controllerNames, mazeSizes);
plotPathLengths(results, controllerNames, mazeSizes);

% New Plots
% plotTimeVsControlSteps(results, controllerNames);
% plotPathLength(results, controllerNames);
% plotVelocityVsTime(results, controllerNames);
plotControlStepsVsMazeSize(results, controllerNames, mazeSizes);


% Function to plot Path Length and Control Steps for each controller (Grouped Bar Chart)

function plotPathLengths(results, controllerNames, mazeSizes)
    % Extract the path lengths for all controllers and maze sizes
    pathLengths = zeros(length(controllerNames), length(mazeSizes));
    
    for c = 1:length(controllerNames)
        controller = controllerNames(c);
        
        for i = 1:length(mazeSizes)
            trajectory = results.(controller)(i).trajectory;  % Get the trajectory data
            % Calculate the distance (path length)
            pathLength = sum(sqrt(diff(trajectory(:,1)).^2 + diff(trajectory(:,2)).^2));  % 2D Euclidean distance
            pathLengths(c, i) = pathLength;
        end
    end
    
    % Create a grouped bar chart for comparison
    figure;
    bar(pathLengths', 'grouped');  % Grouped bar chart
    set(gca, 'XTick', 1:length(mazeSizes), 'XTickLabel', mazeSizes);
    xlabel('Maze Size');
    ylabel('Total Path Length (m)');
    title('Total Path Length per Maze - LQR vs MPC vs SMC Controllers');
    legend(controllerNames, 'Location', 'Best');
    grid on;
end
% Function to plot combined total time per maze for all controllers (Grouped Bar Chart)
function plotCombinedTotalTime(results, controllerNames, mazeSizes)
    % Extract total time data for all controllers
    totalTimes = [];
    for c = 1:length(controllerNames)
        controller = controllerNames(c);
        totalTimes = [totalTimes; arrayfun(@(r) r.totalTime, results.(controller))];
    end
    
    % Create a grouped bar chart for comparison
    figure;
    bar(totalTimes', 'grouped');  % Grouped bar chart
    set(gca, 'XTick', 1:length(mazeSizes), 'XTickLabel', mazeSizes);
    xlabel('Maze Size');
    ylabel('Total Time to Complete (s)');
    title('Total Time per Maze - LQR vs MPC vs SMC vs PID Controllers');
    legend(controllerNames, 'Location', 'Best');
    grid on;
end

% Function to plot combined mean error per maze for all controllers (Grouped Bar Chart)
function plotCombinedMeanError(results, controllerNames, mazeSizes)
    % Extract mean error data for all controllers
    meanErrors = [];
    for c = 1:length(controllerNames)
        controller = controllerNames(c);
        meanErrors = [meanErrors; arrayfun(@(r) r.meanError, results.(controller))];
    end
    
    % Create a grouped bar chart for comparison
    figure;
    bar(meanErrors', 'grouped');  % Grouped bar chart
    set(gca, 'XTick', 1:length(mazeSizes), 'XTickLabel', mazeSizes);
    xlabel('Maze Size');
    ylabel('Mean Tracking Error');
    title('Mean Error per Maze - LQR vs MPC vs SMC vs PID Controllers');
    legend(controllerNames, 'Location', 'Best');
    grid on;
end

% Function to plot Time vs Control Steps for each controller (Scatter Plot)
function plotTimeVsControlSteps(results, controllerNames)
    figure;
    hold on;
    for c = 1:length(controllerNames)
        controller = controllerNames(c);
        totalTime = arrayfun(@(r) r.totalTime, results.(controller));
        controlSteps = arrayfun(@(r) r.numControlSteps, results.(controller));
        % Scatter plot: Added MarkerSize using the correct property
        scatter(controlSteps, totalTime, 50, 'DisplayName', controller, 'LineWidth', 1.5);
    end
    hold off;
    xlabel('Number of Control Steps');
    ylabel('Total Time (s)');
    title('Time vs Control Steps for LQR, MPC, SMC & PID');
    legend('show');
    grid on;
end

% Function to plot Path Length for each controller (Bar Chart)
function plotPathLength(results, controllerNames)
    figure;
    hold on;
    for c = 1:length(controllerNames)
        controller = controllerNames(c);
        totalLength = 0;
        for i = 1:length(results.(controller))
            trajectory = results.(controller)(i).trajectory;  % Get the trajectory
            % Calculate the distance (path length)
            pathLength = sum(sqrt(diff(trajectory(:,1)).^2 + diff(trajectory(:,2)).^2));  % 2D Euclidean distance
            totalLength = totalLength + pathLength;
        end
        bar(c, totalLength, 'DisplayName', controller);
    end
    hold off;
    set(gca, 'XTick', 1:length(controllerNames), 'XTickLabel', controllerNames);
    xlabel('Controller');
    ylabel('Total Path Length (m)');
    title('Total Path Length for LQR, MPC, SMC and PID');
    legend('show');
    grid on;
end

% Function to plot Velocity vs Time for each controller (Line Chart)
function plotVelocityVsTime(results, controllerNames)
    figure;
    hold on;
    
    % Loop through each controller to plot velocity vs time
    for c = 1:length(controllerNames)
        controller = controllerNames(c);
        
        % Initialize a variable to store all velocities
        velocitiesAll = [];
        
        % Loop through each maze size for the current controller
        for i = 1:length(results.(controller))
            % Get the velocity data for the current maze size
            velocities = results.(controller)(i).velocities;
            
            % Check if the velocities data is non-empty and has consistent dimensions
            if ~isempty(velocities) && size(velocities, 2) >= 1
                velocitiesAll = [velocitiesAll; velocities(:, 1)];  % Concatenate the first velocity component
            end
        end
        
        % Create a time vector (assuming the time is continuous)
        time = 1:length(velocitiesAll);
        
        % Plot the velocity data for the controller
        plot(time, velocitiesAll, 'DisplayName', controller, 'LineWidth', 2);
    end
    
    hold off;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Profile for LQR, MPC, SMC and PID');
    legend('show');
    grid on;
end

% Function to plot Control Steps vs Maze Size for each controller (Grouped Bar Chart)
function plotControlStepsVsMazeSize(results, controllerNames, mazeSizes)
    % Extract the number of control steps for each controller
    controlSteps = [];
    for c = 1:length(controllerNames)
        controller = controllerNames(c);
        controlSteps = [controlSteps; arrayfun(@(r) r.numControlSteps, results.(controller))];
    end
    
    % Create a grouped bar chart for comparison
    figure;
    bar(controlSteps', 'grouped');  % Grouped bar chart
    set(gca, 'XTick', 1:length(mazeSizes), 'XTickLabel', mazeSizes);
    xlabel('Maze Size');
    ylabel('Number of Control Steps');
    title('Control Steps per Maze - LQR vs MPC vs SMC vs PID Controllers');
    legend(controllerNames, 'Location', 'Best');
    grid on;
end
