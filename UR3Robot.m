% Running the robot:
baseTr = transl([0, 0, 0]);  % Set the robot's base on the ground

ur3Robot = LinearUR10(baseTr);
title('UR3', 'Color', 'b');
set(gcf, 'Name', 'UR3 robot animation', 'NumberTitle', 'Off');
hold on;
cart1 = PlaceObject('cart.ply',[0.7,-0.75,0.15,0,0,0]);

% Initialize/hardcode brickPickup locations with Z-coordinate set to 0.
brickStart = {
    [0.4, 0.35, 0];  % Adjust Z-coordinate to 0
    [0.3, 0.35, 0];  % Adjust Z-coordinate to 0
    [0.2, 0.35, 0];  % Adjust Z-coordinate to 0
    [0.1, 0.35, 0];  % Adjust Z-coordinate to 0
    [0.0, 0.35, 0];  % Adjust Z-coordinate to 0
    [-0.1, 0.35, 0];  % Adjust Z-coordinate to 0
    [-0.2, 0.35, 0];  % Adjust Z-coordinate to 0
    [-0.3, 0.35, 0];  % Adjust Z-coordinate to 0
    [-0.4, 0.35, 0];  % Adjust Z-coordinate to 0
    % ... (other brick positions)
};

% Initialize/hardcode brickDropoff locations. building a 3x3 wall on the other side
brickDropoff = {
    [0.52, -0.5, 0.25];  % First brick dropoff position (adjust Z-coordinate as needed)
    [0.42, -0.5, 0.25];  % Adjust Z-coordinate
    [0.3, -0.5, 0.25];   % Adjust Z-coordinate
    [0.5, -0.5, 0.3];    % Adjust Z-coordinate
    [0.4, -0.5, 0.3];    % Adjust Z-coordinate
    [0.3, -0.5, 0.3];    % Adjust Z-coordinate
    [0.5, -0.5, 0.35];   % Adjust Z-coordinate
    [0.4, -0.5, 0.35];   % Adjust Z-coordinate
    [0.3, -0.5, 0.35];   % Adjust Z-coordinate
};
for i = 1:length(brickStart)
    brick_position = brickStart{i};
    xyz = brick_position(1:3);
    h_brick = PlaceObject('HalfSizedRedGreenBrick.ply', xyz); % Store handle

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
    PlaceObject('HalfSizedRedGreenBrick.ply', dropoff);

    drawnow();
end













 %         it is all working up till here      %
  
%    so below is a class implementation which i tried but unfortunately it
%    did not work
        
%  classdef UR3Robot
%     properties
%         model
%         environment % structure to store objects like tables, cones, etc.
%         bricks % structure to store brick data
%         brickStart
%         brickDropoff
%     end
% 
%     methods
%         % Constructor
%         function obj = UR3Robot(baseTr)
%             obj.model = LinearUR3(baseTr);
%             obj.environment = struct();
%             obj.bricks = struct('start', {}, 'end', {});
%             obj = setupEnvironment(obj);
%             obj = initializeBrickLocations(obj);
%         end
% 
%         % Set up environment
%         function obj = setupEnvironment(obj)
%             title('UR3','Color','b');
%             set(gcf, 'Name', 'UR3 robot animation', 'NumberTitle', 'Off');
%             hold on;
%             surf([-1.8,-1.8;1.8,1.8], [-1.8,1.8;-1.8,1.8], [0.01,0.01;0.01,0.01], 'CData',imread('concrete (1).jpg'), 'FaceColor','texturemap');
% 
%             obj.environment.fireE = PlaceObject('fireExtinguisher.ply',[0.6,0.5,0.1,0,0,0]);
%             obj.environment.table = PlaceObject('table.ply',[-0.2,0,0.25,0,0,0]);
%             obj.environment.f_1 = PlaceObject('fence.ply',[-0.2,0.8,0.15,0,0,0]);
%             obj.environment.f_2 = PlaceObject('fence.ply',[0.5,0.8,0.15,0,0,0]);
%             obj.environment.f_3 = PlaceObject('fence.ply',[-0.2,-0.8,0.15,0,0,0]);
%             obj.environment.f_4 = PlaceObject('fence.ply',[0.5,-0.8,0.15,0,0,0]);
%             obj.environment.cone_1 = PlaceObject('cone.ply',[0.7,-0.4,0,0,0,0]);
%             obj.environment.cone_2 = PlaceObject('cone.ply',[0.7,0.4,0,0,0,0]);
%             obj.environment.man = PlaceObject('man.ply',[-0.6,-0.5,0.25,0,0,0]);
%         end
% 
%         % Initialize brick locations
%         function obj = initializeBrickLocations(obj)
%             obj.brickStart = {
%                 [0.4,0.35,0.3,0,0,0];
%                 [0.3,0.35,0.3,0,0,0];
%                 [0.2,0.35,0.3,0,0,0];
%                 [0.1,0.35,0.3,0,0,0];
%                 [0.0,0.35,0.3,0,0,0];
%                 [-0.1,0.35,0.3,0,0,0];
%                 [-0.2,0.35,0.3,0,0,0];
%                 [-0.3,0.35,0.3,0,0,0];
%                 [-0.4,0.35,0.3,0,0,0];
%                 % ... (other brick positions)
%             };
% 
%             obj.brickDropoff = {
%                 [0.5, -0.3, 0.3];
%                 [0.4, -0.3, 0.3];
%                 [0.3, -0.3, 0.3];
%                 [0.5, -0.3, 0.35];
%                 [0.4, -0.3, 0.35];
%                 [0.3, -0.3, 0.35];
%                 [0.5, -0.3, 0.4];
%                 [0.4, -0.3, 0.4];
%                 [0.3, -0.3, 0.4];
%             };
% 
%             for i = 1:length(obj.brickStart)
%                obj.bricks(i).start = obj.brickStart{i};
%                obj.bricks(i).end = obj.brickDropoff{i};
%            end
%         end
% 
%         function moveBricks(obj)
%             for i = 1:length(obj.bricks)
%                 brick_position = obj.bricks(i).start;
%                 xyz = brick_position(1:3);
%                 h_brick = PlaceObject('HalfSizedRedGreenBrick.ply', xyz);
% 
%                 % Define pickup and dropoff locations
%                 pickup = brick_position(1:3);
%                 dropoff = obj.bricks(i).end;  
% 
%                 T_pickup = transl(pickup)*rpy2tr(0,pi,0);
%                 T_dropoff = transl(dropoff)*rpy2tr(0, pi, 0);
% 
%                 q_current = obj.model.getpos();
%                 q_pickup = obj.model.ikcon(T_pickup);
%                 qMatrix_pickup = jtraj(q_current, q_pickup, 50);
% 
%                 % Animate pickup
%                 for j = 1:50
%                     obj.model.animate(qMatrix_pickup(j, :));
%                     drawnow();
%                 end
% 
%                 % "Remove" brick from original position by making it invisible
%                 delete(h_brick);
% 
%                 q_dropoff = obj.model.ikcon(T_dropoff);
%                 qMatrix_dropoff = jtraj(q_pickup, q_dropoff, 50);
% 
%                 % Animate moving to dropoff
%                 for j = 1:50
%                     obj.model.animate(qMatrix_dropoff(j, :));
%                     drawnow();
%                 end
% 
%                 % Display the brick at the new location after robot "drops" it off
%                 PlaceObject('HalfSizedRedGreenBrick.ply', dropoff);
% 
%                 drawnow();
%             end
%         end
%     end
% end
% 
