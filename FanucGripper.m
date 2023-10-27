close all;
% Load Fanuc Robot and Gripper
import robotics.*;
robot = loadrobot('fanucm16ib');
gripper = ThreeFingerGripper();

% Define Open and Closed Joint Angles for Gripper
gripperOpen = [5*pi/6 -pi/6 -pi/4; pi/6 pi/6 pi/4; pi/6 pi/6 pi/4];
gripperClosed = [5*pi/8 -pi/6 -pi/12; 3*pi/8 pi/6 pi/12; 3*pi/8 pi/6 pi/12];

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
% Set the DataFormat property of the robot
robot.DataFormat = 'row';

% Define robot configurations for pick and place operation
configPick = homeConfiguration(robot);
configPlace = randomConfiguration(robot);

% Convert configurations to vectors of joint angles
qPick = [configPick.JointPosition];
qPlace = [configPlace.JointPosition];

% Generate joint space trajectory
steps = 50; % adjust as needed
[qTraj,~,~] = jtraj(qPick, qPlace, steps);

% Animate Robot and Gripper
for t = 1:steps
    % Get current configuration from trajectory
    qCurrent = qTraj(t,:);
    
    % Convert vector of joint angles back into configuration structure
    configCurrent = configPick; % use home configuration as a template
    for i = 1:length(configCurrent)
        configCurrent(i).JointPosition = qCurrent(i);
    end
    
    % Check for self-collision
    if ~robot.checkCollision(qCurrent)
        % Update end effector frame based on current robot configuration
        endEffectorFrame = getTransform(robot, configCurrent, 'link_6');
        
        % Interpolate between open and closed configurations for the gripper
        gripperState = (1-t/steps)*gripperOpen + (t/steps)*gripperClosed;
        
        for j = 1:length(gripper.model)
            % Update base property before each animation step
            gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
            gripper.model{j}.animate(gripperState(j,:));
        end
        
        % Update robot configuration in the plot
        show(robot, configCurrent, 'Frames', 'off', 'PreservePlot', false);
        drawnow; % Force MATLAB to update the plot
    else
        disp('Collision detected. Skipping this configuration.');
    end
end