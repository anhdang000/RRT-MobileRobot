clear
% All dimensions are in meter
% Rectangle are shaped by width and height
%% Initialize environment
[start, goal, mapSize, mapMatrix, obstacles] = initMap();

%% Path planning
rrt = RRTGraph(start, goal, mapMatrix, mapSize);

biasIter = 4;
iter = 0;
while ~rrt.goalFlag
    if mod(iter, biasIter) == 0
        rrt = rrt.bias(goal);
        plot(rrt.treeCoors(end, 1), rrt.treeCoors(end, 2), 'c.');
    else
        rrt = rrt.expand();
        plot(rrt.treeCoors(end, 1), rrt.treeCoors(end, 2), 'c.');
    end
    
    rrt = rrt.getPath2Goal();
    
    iter = iter + 1;
    drawnow
end

pathCoors = rrt.getPathCoors();
plot(pathCoors(:, 1), pathCoors(:, 2), 'k-.')

%% Optimize waypoint
optimalPathCoors = rrt.optimizePath(pathCoors);
plot(optimalPathCoors(:, 1), optimalPathCoors(:, 2), ...
    'LineWidth', 2, 'Color', [0 0.4470 0.7410]);
path = optimalPathCoors;

saveas(gcf,'assets/RRT_result.png')

pause(1);

%% Robot initialization and Path following
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% Define path following controller
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.2; % m/s
controller.MaxAngularVelocity = 2; % rad/s
controller.LookaheadDistance = 0.3;

% Drive the Robot over the Desired Waypoints
goalRadius = 0.05;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);


% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/4;

trajectory = [];
while( distanceToGoal > goalRadius )
    trajectory = [trajectory; robotCurrentPose(1:2)'];
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(pathCoors(:, 1), pathCoors(:, 2), 'k-.');
    axis equal
    grid on
    hold all
    plot(path(:, 1), path(:, 2), 'LineWidth', 2, 'Color', [0 0.4470 0.7410]);
    drawMapInLoop(start, goal, obstacles);
    plot(trajectory(:, 1), trajectory(:, 2), ...
    'LineWidth', 2, 'Color', [0.3010 0.7450 0.9530]);

    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 4]);
    ylim([0 4]);
    xticks(0:1:4);
    yticks(0:1:4);
    waitfor(vizRate);
end

saveas(gcf,'assets/final_result.png')
