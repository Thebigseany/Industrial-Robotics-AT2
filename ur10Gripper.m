close all;

import robotics.*;

% Load UR10 Robot
ur10 = LinearUR10();

% Manually adjust joint limits for prismatic joint
ur10.model.links(1).qlim = [0 0.8];

% Initialize Gripper
gripper = ThreeFingerGripper();

% Define Open and Closed Joint Angles for Gripper
gripperOpen = [5*pi/6 -pi/6 -pi/4; pi/6 pi/6 pi/4; pi/6 pi/6 pi/4];
gripperClosed = [5*pi/8 -pi/6 -pi/12; 3*pi/8 pi/6 pi/12; 3*pi/8 pi/6 pi/12];

hold on;
axis([-3 3 -3 3 -1 3]);

% Plot UR10
homeConfig = [0, 0, 0, 0, 0, 0, 0];
ur10.model.plot(homeConfig);

% Define a target pose for the gripper
targetPose = transl(0.5, 0.5, 0.2) * trotx(pi/2);

% Use IKcon to find the joint angles to reach the target pose
weights = [1 1 1 1 1 1];  % Adjust these weights as necessary
[qTarget, qError] = ikcon(ur10.model, targetPose);

if qError > 0.1 % adjust threshold as needed
    disp('Solution is not accurate enough.');
end

% Plot Gripper on UR10 End Effector
basePositions = [0, 0.04, 0; 0, -0.04, -0.07; 0, -0.04, 0.07];
endEffectorFrame = ur10.model.fkine(homeConfig);

for j = 1:length(gripper.model)
    T_trans = transl(basePositions(j,:));
    T_rotz = trotz(-pi/2);
    gripper.model{j}.base = endEffectorFrame * T_trans * T_rotz;
    gripper.model{j}.plot(gripperOpen(j,:));
end

% Animate UR10 and Gripper
for t = 0:0.05:1
    currentQ = (1 - t) * homeConfig + t * qTarget;
    ur10.model.animate(currentQ);
    endEffectorFrame = ur10.model.fkine(currentQ);
    
    % Interpolate between open and closed configurations for opening
    gripperState = (1-t)*gripperOpen + t*gripperClosed;
    
    for j = 1:length(gripper.model)
        gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
        gripper.model{j}.animate(gripperState(j,:));
    end
    
    % Collision Detection
    if ur10.model.checkCollision(currentQ)
        disp('Collision Detected!');
        break;
    end
    
    drawnow; % Force MATLAB to update the plot
end

for t = 0:0.05:1
    % Interpolate between closed and open configurations for closing
    gripperState = (1-t)*gripperClosed + t*gripperOpen;
    
    for j = 1:length(gripper.model)
        gripper.model{j}.base = (endEffectorFrame * transl(basePositions(j,:)) * trotz(-pi/2));
        gripper.model{j}.animate(gripperState(j,:));
    end
    
    % Collision Detection
    if ur10.model.checkCollision(homeConfig)
        disp('Collision Detected!');
        break;
    end
    
    drawnow; % Force MATLAB to update the plot
end
