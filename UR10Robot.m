close all;
hold on;
% Running the robot:
baseTr = transl([3, 1, 0]);  % Set the robot's base on the ground
import robotics.*;

ur3Robot = LinearUR10(baseTr);
robot = loadrobot('fanucm16ib');
gripper = ThreeFingerGripper();

title('UR10', 'Color', 'b');
set(gcf, 'Name', 'UR10 robot animation', 'NumberTitle', 'Off');
%% 
% Environment setup 
surf([-2,-2;4.5,4.5] ...
,[-2,4.5;-2,4.5] ...
,[0.01,0.01;0.01,0.01] ...
,'CData',imread('concrete (1).jpg') ...
,'FaceColor','texturemap');
fireE = PlaceObject('fireExtinguisher.ply',[0.6,0.5,0.1,0,0,0]);

f_1 = PlaceObject('fence.ply',[3,2,0.15,0,0,0]);
f_2 = PlaceObject('fence.ply',[2,2,0.15,0,0,0]);
f_3 = PlaceObject('fence.ply',[3,-0.5,0.15,0,0,0]);
f_4 = PlaceObject('fence.ply',[2,-0.5,0.15,0,0,0]);
cone_1 = PlaceObject('cone.ply',[4,0.5,0,0,0,0]);
cone_2 = PlaceObject('cone.ply',[4,1.5,0,0,0,0]);
man = PlaceObject('man.ply',[-0.6,-0.5,0.25,0,0,0]);
cart1 = PlaceObject('cart.ply',[3.75,0.25,0.15,0,0,0]);
%% 

 % brickPickup locations with Z-coordinate set to 0.

 brickHandlesOnCart = [];  % To store the handles of bricks on the cart
brickStart = {
    [3.4, 1.35, 0];  % Adjust Z-coordinate to 0
    [3.3, 1.35, 0];  % Adjust Z-coordinate to 0
    [3.2, 1.35, 0];  % Adjust Z-coordinate to 0
    [3.1, 1.35, 0];  % Adjust Z-coordinate to 0
    [3.0, 1.35, 0];  % Adjust Z-coordinate to 0
    [2.9, 1.35, 0];  % Adjust Z-coordinate to 0
    [2.8, 1.35, 0];  % Adjust Z-coordinate to 0
    [2.7, 1.35, 0];  % Adjust Z-coordinate to 0
    [2.6, 1.35, 0];  % Adjust Z-coordinate to 0
    % ... (other brick positions)
};

% Initialize/hardcode brickDropoff locations. building a 3x3 wall on the other side
brickDropoff = {
    [3.52, 0.5, 0.25];  % First brick dropoff position (adjust Z-coordinate as needed)
    [3.42, 0.5, 0.25];  % Adjust Z-coordinate
    [3.3, 0.5, 0.25];   % Adjust Z-coordinate
    [3.5, 0.5, 0.3];    % Adjust Z-coordinate
    [3.4, 0.5, 0.3];    % Adjust Z-coordinate
    [3.3, 0.5, 0.3];    % Adjust Z-coordinate
    [3.5, 0.5, 0.35];   % Adjust Z-coordinate
    [3.4, 0.5, 0.35];   % Adjust Z-coordinate
    [3.3, 0.5, 0.35];   % Adjust Z-coordinate
};

brickUnload = {
    [0.8, 1.05, 0.25];  % First brick unload position (adjust Z-coordinate as needed)
    [1.0, 1.05, 0.25];  % Adjust Z-coordinate
    [1.2, 1.05, 0.25];   % Adjust Z-coordinate
    [0.8, 1.05, 0.3];   % Adjust Z-coordinate
    [1.0, 1.05, 0.3];   % Adjust Z-coordinate
    [1.2, 1.05, 0.3];    % Adjust Z-coordinate
    [0.8, 1.05, 0.35];  % Adjust Z-coordinate
    [1.0, 1.05, 0.35];  % Adjust Z-coordinate
    [1.2, 1.05, 0.35];   % Adjust Z-coordinate
};

%% Define Fanuc and Gripper
% Define Open and Closed Joint Angles for Gripper
gripperOpen = [5*pi/6 -pi/6 -pi/4; pi/6 pi/6 pi/4; pi/6 pi/6 pi/4];

gripperClosed = [5*pi/8 -pi/6 -pi/12; 3*pi/8 pi/6 pi/12; 3*pi/8 pi/6 pi/12]; % Will adjust this value once Ali provides dimensions for the ore model
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

    q_current = ur3Robot.model.getpos();
    q_pickup = ur3Robot.model.ikcon(T_pickup);
    qMatrix_pickup = jtraj(q_current, q_pickup, 50);

    % Animate pickup
    for j = 1:50
        ur3Robot.model.animate(qMatrix_pickup(j, :));
        drawnow();
    end

    % "Remove" brick from the original position by making it invisible
    delete(h_brick);

    q_dropoff = ur3Robot.model.ikcon(T_dropoff);
    qMatrix_dropoff = jtraj(q_pickup, q_dropoff, 50);

    % Animate moving to dropoff
    for j = 1:50
        ur3Robot.model.animate(qMatrix_dropoff(j, :));
        drawnow();
    end

    % Display the brick at the new location after the robot "drops" it off
    PlaceObject('ore.ply', dropoff);

    % When placing the brick on the cart
    h_brickOnCart = PlaceObject('ore.ply', dropoff);
    brickHandlesOnCart(i) = h_brickOnCart;
    drawnow();
end

%% cart movement


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
    q_current = ur3Robot.model.getpos();
    q_pickFromCart = ur3Robot.model.ikcon(T_pickup);
    qMatrix_pickFromCart = jtraj(q_current, q_pickFromCart, 50);

    % Animate robot reaching the brick in the cart
    for j = 1:50
        ur3Robot.model.animate(qMatrix_pickFromCart(j, :));
        drawnow();
    end

    % Delete the brick from the cart
    delete(brickHandlesOnCart(i));

    q_unload = ur3Robot.model.ikcon(T_unload);
    qMatrix_unload = jtraj(q_pickFromCart, q_unload, 50);

    % Animate robot placing the brick at the unload position
    for j = 1:50
        ur3Robot.model.animate(qMatrix_unload(j, :));
        drawnow();
    end

    % Display the brick at the unload location after the robot "drops" it off
    PlaceObject('ore.ply', unload);
    drawnow();
end












