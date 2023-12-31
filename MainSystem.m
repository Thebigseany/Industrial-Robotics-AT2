close all;
hold on;
% Running the robot:
baseTr = transl([3, 1, 0]);  % Set the robot's base on the ground
import robotics.*;

ur10Robot = LinearUR10(baseTr);
robot = loadrobot('fanucm16ib');
gripper = ThreeFingerGripper();

title('UR10', 'Color', 'b');
set(gcf, 'Name', 'UR10 robot animation', 'NumberTitle', 'Off');
%% 
% Environment setup 
surf([-2,-2;6,6] ...
,[-2,6;-2,6] ...
,[0.01,0.01;0.01,0.01] ...
,'CData',imread('concrete (1).jpg') ...
,'FaceColor','texturemap');

% Load your wall image
wallImage = imread('Wall.jpg');

% Left vertical wall
surf([-2,-2;-2,-2], ...
     [-2,-2;6,6], ...
     [0,2;0,2], ...
     'CData', wallImage, ...
     'FaceColor', 'texturemap');

% Right vertical wall
surf([6,6;6,6], ...
     [-2,-2;6,6], ...
     [0,2;0,2], ...
     'CData', wallImage, ...
     'FaceColor', 'texturemap');

% Bottom vertical wall
surf([-2,6; -2,6], ...
     [-2,-2; -2,-2], ...
     [0,0; 2,2], ...
     'CData', wallImage, ...
     'FaceColor', 'texturemap');

% Top vertical wall
surf([-2,6; -2,6], ...
     [6,6; 6,6], ...
     [0,0; 2,2], ...
     'CData', wallImage, ...
     'FaceColor', 'texturemap');


StopButton = PlaceObject('emergencyStopButton.ply',[0,2,0.1,0,0,0]);


fireE1 = PlaceObject('fireExtinguisher.ply',[-1,2,0.1,0,0,0]);
fireE2 = PlaceObject('fireExtinguisher.ply',[3.8,2,0.1,0,0,0]);
fireE3 = PlaceObject('fireExtinguisher.ply',[3.8,-0.6,0.1,0,0,0]);
fireE4 = PlaceObject('fireExtinguisher.ply',[-1,-0.6,0.1,0,0,0]);

f_1 = PlaceObject('fence.ply',[3.4,2.5,0.15,0,0,0]);
f_2 = PlaceObject('fence.ply',[2.4,2.5,0.15,0,0,0]);
f_3 = PlaceObject('fence.ply',[1.4,2.5,0.15,0,0,0]);
f_4 = PlaceObject('fence.ply',[0.4,2.5,0.15,0,0,0]);
f_5 = PlaceObject('fence.ply',[-0.6,2.5,0.15,0,0,0]);

fr_1 = PlaceObject('rotatedFence.ply', [-1.5,2,0.15,0,0,0]);
fr_2 = PlaceObject('rotatedFence.ply', [-1.5,1,0.15,0,0,0]);
fr_3 = PlaceObject('rotatedFence.ply', [-1.5,0,0.15,0,0,0]);
fr_4 = PlaceObject('rotatedFence.ply', [-1.5,-1,0.15,0,0,0]);
fr_5 = PlaceObject('rotatedFence.ply', [4.2,2,0.15,0,0,0]);
fr_6 = PlaceObject('rotatedFence.ply', [4.2,1,0.15,0,0,0]);
fr_7 = PlaceObject('rotatedFence.ply', [4.2,0,0.15,0,0,0]);
fr_8 = PlaceObject('rotatedFence.ply', [4.2,-1,0.15,0,0,0]);

f_6 = PlaceObject('fence.ply',[3.4,-1.2,0.15,0,0,0]);
f_7 = PlaceObject('fence.ply',[2.4,-1.2,0.15,0,0,0]);
f_8 = PlaceObject('fence.ply',[1.4,-1.2,0.15,0,0,0]);
f_9 = PlaceObject('fence.ply',[0.4,-1.2,0.15,0,0,0]);
f_10 = PlaceObject('fence.ply',[-0.6,-1.2,0.15,0,0,0]);

cone_1 = PlaceObject('cone.ply',[2.5,3,0,0,0,0]);
cone_2 = PlaceObject('cone.ply',[1,3,0,0,0,0]);
cone_3 = PlaceObject('cone.ply',[-0.5,3,0,0,0,0]);
cone_4 = PlaceObject('cone.ply',[2.5,-1.6,0,0,0,0]);
cone_5 = PlaceObject('cone.ply',[1,-1.6,0,0,0,0]);
cone_6 = PlaceObject('cone.ply',[-0.5,-1.6,0,0,0,0]);


man = PlaceObject('man.ply',[-0.6,1,0.25,0,0,0]);
man2 = PlaceObject('man.ply',[3.6,1,0.25,0,0,0]);
man3 = PlaceObject('man2.ply',[3.6,0.75,0.35,0,0,0]);
man4 = PlaceObject('man2.ply',[-0.8,0.75,0.35,0,0,0]);

cart1 = PlaceObject('cart.ply',[3.75,0.25,0.15,0,0,0]);
%% 

 % brickPickup locations with Z-coordinate set to 0.

 brickHandlesOnCart = [];  % To store the handles of bricks on the cart
brickStart = {
    [3.4, 1.35, 0];
    [3.3, 1.35, 0];
    [3.2, 1.35, 0];
    [3.1, 1.35, 0];
};

% Initialize/hardcode brickDropoff locations. building a 3x3 wall on the other side
brickDropoff = {
    [3.52, 0.5, 0.25];  % First brick dropoff position
    [3.42, 0.5, 0.25];
    [3.3, 0.5, 0.25];
    [3.5, 0.5, 0.35];

};

brickUnload = {
    [0.8, 1.05, 0];  % First brick unload position 
    [1.0, 1.05, 0];
    [1.2, 1.05, 0]; 
    [0.8, 1.05, 0.1];

};

%% Define Fanuc and Gripper
% Define Open and Closed Joint Angles for Gripper
gripperOpen = [5*pi/6 -pi/6 -pi/4; pi/6 pi/6 pi/4; pi/6 pi/6 pi/4];

gripperClosed = [5*pi/8 -pi/6 -pi/12; 3*pi/8 pi/6 pi/12; 3*pi/8 pi/6 pi/12];
hold on;
% axis([-3 3 -3 3 0 3]);

% Plot robot and Gripper
show(robot); % plot robot and gripper

% Plot Gripper on Robot End Effector
basePositions = [0, 0.04, 0; 0, -0.04, -0.07; 0, -0.04, 0.07];
endEffectorFrame = getTransform(robot, robot.homeConfiguration, 'link_6');
for j = 1:length(gripper.model)
    gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
    gripper.model{j}.plot(gripperOpen(j,:));
end

%% Ore Generation
for i = 1:length(brickStart)
    brick_position = brickStart{i};
    xyz = brick_position(1:3);
    h_brick = PlaceObject('ore.ply', xyz); % Store handle
    
    % Define pickup and dropoff locations
    pickup = brick_position(1:3);
    dropoff = brickDropoff{i};

    T_pickup = transl(pickup) * rpy2tr(0, pi, 0);
    T_dropoff = transl(dropoff) * rpy2tr(0, pi, 0);

    q_current = ur10Robot.model.getpos();
    q_pickup = ur10Robot.model.ikcon(T_pickup);
    qMatrix_pickup = jtraj(q_current, q_pickup, 50);

    % Animate pickup
    for j = 1:50
        ur10Robot.model.animate(qMatrix_pickup(j, :));
        drawnow();
    end

    % "Remove" brick from the original position by making it invisible
    delete(h_brick);

    q_dropoff = ur10Robot.model.ikcon(T_dropoff);
    qMatrix_dropoff = jtraj(q_pickup, q_dropoff, 50);

    % Animate moving to dropoff
    for j = 1:50
        ur10Robot.model.animate(qMatrix_dropoff(j, :));
        drawnow();
        
    end
    
    % Display the brick at the new location after the robot "drops" it off
    PlaceObject('ore.ply', dropoff);

    % When placing the brick on the cart
    h_brickOnCart = PlaceObject('ore.ply', dropoff);
    brickHandlesOnCart(i) = h_brickOnCart;
    
    
    drawnow();
end

%% cart movement with ores


% Define the initial and final positions of the cart
cartInitialPosition = [3.75,0.25,0.15,0,0,0];
cartFinalPosition = [2.00,0.25,0.15,0,0,0];
 
% Number of steps for the animation
numSteps = 25;
 
% Compute the step increments for the animation
stepIncrement = (cartFinalPosition - cartInitialPosition) / numSteps;
 
% Animate the cart movement
for step = 1:numSteps
    % Compute the new position of the cart for this step
    newCartPosition = cartInitialPosition + stepIncrement * step;
    
    % Delete the cart's current representation
    delete(cart1);
    
    % Place the cart at the new position
    cart1 = PlaceObject('cart.ply', newCartPosition);
    
    % Move each brick on the cart as well
    for m = 1:length(brickHandlesOnCart)
        % Calculate the position change for each brick relative to the cart
        brickDeltaPosition = newCartPosition - cartInitialPosition;
        
        % Delete the current brick representation
        delete(brickHandlesOnCart(m));
        
        % Use the known position from brickDropoff instead of fetching it from a handle
        newBrickPosition = brickDropoff{m} + brickDeltaPosition(1:3);
        
        % Place the brick at its new position
        brickHandlesOnCart(m) = PlaceObject('ore.ply', newBrickPosition);
    end
    
    % Draw the updated scene
    drawnow();
end

% Create a loop for unloading the bricks and placing them at the brickUnload positions
for i = 1:length(brickUnload)
    pickup1 = brickDropoff{i} + (cartFinalPosition(1:3) - cartInitialPosition(1:3)); % Picking from the cart's final position
    unload = brickUnload{i};

    T_pickup = transl(pickup1) * rpy2tr(0, pi, 0);
    T_unload = transl(unload) * rpy2tr(0, pi, 0);
    q_current = ur10Robot.model.getpos();
    q_pickFromCart = ur10Robot.model.ikcon(T_pickup);
    qMatrix_pickFromCart = jtraj(q_current, q_pickFromCart, 50);

    % Animate robot reaching the brick in the cart
    for j = 1:50
        ur10Robot.model.animate(qMatrix_pickFromCart(j, :));
        drawnow();
    end

    % Delete the brick from the cart
    delete(brickHandlesOnCart(i));

    q_unload = ur10Robot.model.ikcon(T_unload);
    qMatrix_unload = jtraj(q_pickFromCart, q_unload, 50);

    % Animate robot placing the brick at the unload position
    for j = 1:50
        ur10Robot.model.animate(qMatrix_unload(j, :));
        drawnow();
    end

    % Display the brick at the unload location after the robot "drops" it off
    PlaceObject('ore.ply', unload);
    drawnow();
end

%% Fanuc M16iB Movement
gripperOpen = [5*pi/6 -pi/6 -pi/4; pi/6 pi/6 pi/4; pi/6 pi/6 pi/4];

gripperClosed = [5*pi/7 -pi/6 -pi/12; 2*pi/7 pi/6 pi/12; 2*pi/7 pi/6 pi/12];

% Set the DataFormat property of the robot
robot.DataFormat = 'row';

% Define robot configurations for pick and place operation
qPick = robot.homeConfiguration;
% qPlace = robot.randomConfiguration; % REPLACE with joint configuration of where the ore will be placed (currently generates a random configuration)

% Robot Movement to second position
% Gripper Opens
for t_2 = 0:0.05:1
    % Interpolate between open and closed configurations
    gripperState = (1-t_2)*gripperOpen + t*gripperClosed;

    for j = 1:length(gripper.model)
        % Update base property before each animation step
        gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
        gripper.model{j}.animate(gripperState(j,:));
    end

    drawnow; % Force MATLAB to update the plot
end

home = robot.homeConfiguration;
guess = robot.randomConfiguration;
weights = [pi, pi, pi, 2, 2, 1];
for p = 1: length(brickUnload)

    ik = inverseKinematics('RigidBodyTree',robot);
    [configSol,~] = ik('link_6',brickUnload{p,:}, weights, guess);
    qPlace = configSol;
    [qTraj,~,~] = jtraj(qPick, qPlace, steps);

    
    for t_1 = 1:steps
        % Get current configuration from trajectory
        qCurrent = qTraj(t_1,:);
        
        % Check for self-collision
        if ~robot.checkCollision(qCurrent, 'SkippedSelfCollisions', 'parent')
            % Update end effector frame based on current robot configuration
            endEffectorFrame = getTransform(robot, qCurrent, 'link_6');
            
            % Interpolate between open and closed configurations for the gripper
            gripperState = (1-t_1/steps)*gripperOpen + (t_1/steps)*gripperClosed;
            
            % Update robot configuration in the plot
             show(robot, qCurrent, 'Frames', 'off', 'PreservePlot', false);
             
            drawnow; % Force MATLAB to update the plot`
        end 
    end
    
    for g_1 = 0:0.05:1
        % Interpolate between open and closed configurations
        gripperState = (1-g_1)*gripperClosed + t*gripperOpen;
    
        for j = 1:length(gripper.model)
            % Update base property before each animation step
            gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
            gripper.model{j}.animate(gripperState(j,:));
        end
    
        drawnow; % Force MATLAB to update the plot
    end

    [configSol,~] = ik('link_6',orePlace2{p,:},guess);
    qPlace = configSol;
    [qTraj,~,~] = jtraj(qStart, qPlace, steps);

    for t_2 = 1:steps
        % Get current configuration from trajectory
        qCurrent = qTraj(t_2,:);
        
        % Check for self-collision
        if ~robot.checkCollision(qCurrent, 'SkippedSelfCollisions', 'parent')
            % Update end effector frame based on current robot configuration
            endEffectorFrame = getTransform(robot, qCurrent, 'link_6');
            
            % Interpolate between open and closed configurations for the gripper
            gripperState = (1-t_2/steps)*gripperOpen + (t_2/steps)*gripperClosed;
            
            % Update robot configuration in the plot
             show(robot, qCurrent, 'Frames', 'off', 'PreservePlot', false);
             
            drawnow; % Force MATLAB to update the plot`
        end 
    end
    
    % Animate Gripper
    for g_2 = 0:0.05:1
        % Interpolate between open and closed configurations
        gripperState = (1-g_2)*gripperOpen + t*gripperClosed;
    
        for j = 1:length(gripper.model)
            % Update base property before each animation step
            gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
            gripper.model{j}.animate(gripperState(j,:));
        end
    
        drawnow; % Force MATLAB to update the plot
    end
    
    [configSol,~] = ik('link_6',home,guess);
    qPlace = configSol;
    [qTraj,~,~] = jtraj(qStart, qPlace, steps);
    
    for t_3 = 1:steps
        % Get current configuration from trajectory
        qCurrent = qTraj(t_3,:);
        
        % Check for self-collision
        if ~robot.checkCollision(qCurrent, 'SkippedSelfCollisions', 'parent')
            % Update end effector frame based on current robot configuration
            endEffectorFrame = getTransform(robot, qCurrent, 'link_6');
            
            % Interpolate between open and closed configurations for the gripper
            gripperState = (1-t_3/steps)*gripperOpen + (t_3/steps)*gripperClosed;
            
            % Update robot configuration in the plot
             show(robot, qCurrent, 'Frames', 'off', 'PreservePlot', false);
             
            drawnow; % Force MATLAB to update the plot`
        end 
    end
end











