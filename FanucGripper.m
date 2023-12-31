close all;
% Load Fanuc Robot and Gripper
import robotics.*;
robot = loadrobot('fanucm16ib');
gripper = ThreeFingerGripper();

% Define Open and Closed Joint Angles for Gripper
gripperOpen = [5*pi/6 -pi/6 -pi/4; pi/6 pi/6 pi/4; pi/6 pi/6 pi/4];

gripperClosed = [5*pi/8 -pi/6 -pi/12; 3*pi/8 pi/6 pi/12; 3*pi/8 pi/6 pi/12]; % Will adjust this value once Ali provides dimensions for the ore model
hold on;
axis([-3 3 -3 3 -1 3]);

% Plot robot and Gripper
show(robot); % plot robot and gripper

% Plot Gripper on Robot End Effector
basePositions = [0, 0.04, 0; 0, -0.04, -0.07; 0, -0.04, 0.07];
endEffectorFrame = getTransform(robot, robot.homeConfiguration, 'link_6');
for j = 1:length(gripper.model)
    gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
    gripper.model{j}.plot(gripperOpen(j,:));
end

%% Animate Gripper Opening
% Animate Gripper
for t = 0:0.05:1
    % Interpolate between open and closed configurations
    gripperState = (1-t)*gripperOpen + t*gripperClosed;
    
    for j = 1:length(gripper.model)
        % Update base property before each animation step
        gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
        gripper.model{j}.animate(gripperState(j,:));
    end
    
    drawnow; % Force MATLAB to update the plot
end

%% Animate Gripper Closing
% Animate Gripper
for t = 0:0.05:1
    % Interpolate between open and closed configurations
    gripperState = (1-t)*gripperClosed + t*gripperOpen;
    
    for j = 1:length(gripper.model)
        % Update base property before each animation step
        gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
        gripper.model{j}.animate(gripperState(j,:));
    end
    
    drawnow; % Force MATLAB to update the plot
end

%% Move from current position to position of next ore (TBC)
% get position of next ore
% jtraj to get q of next ore 
% qpath from current pose to next ore pose

% Set the DataFormat property of the robot
robot.DataFormat = 'row';

% Define robot configurations for pick and place operation
qPick = robot.homeConfiguration;
qPlace = robot.randomConfiguration; % REPLACE with joint configuration of where the ore will be placed (currently generates a random configuration)

% Generate joint space trajectory
steps = 100; % adjust as needed
animationDuration = 5; %5 seconds
frameDelay = animationDuration/steps;
[qTraj,~,~] = jtraj(qPick, qPlace, steps);

% Animate Robot and Gripper
try
    for t = 1:steps
        % Get current configuration from trajectory
        qCurrent = qTraj(t,:);
        
        % Check for self-collision
        if ~robot.checkCollision(qCurrent, 'SkippedSelfCollisions', 'parent')
            % Update end effector frame based on current robot configuration
            endEffectorFrame = getTransform(robot, qCurrent, 'link_6');
            
            % Interpolate between open and closed configurations for the gripper
            gripperState = (1-t/steps)*gripperOpen + (t/steps)*gripperClosed;
            
            for j = 1:length(gripper.model)
                % Update base property before each animation step
                gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
                gripper.model{j}.animate(gripperState(j,:));
            end
            
            % Update robot configuration in the plot
             show(robot, qCurrent, 'Frames', 'off', 'PreservePlot', false);
            % drawnow; % Force MATLAB to update the plot
        end
    end
catch ME
end
