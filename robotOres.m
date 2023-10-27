classdef robotOres < handle
    %ROBOTOres A class that creates a herd of robot Ores
	%   The Ores can be moved around randomly. It is then possible to query
    %   the current location (base) of the Ores.    
    
    %#ok<*TRYNC>    

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 1;
    end
    
    properties
        %> Number of Ores
        oreCount = 2;

        %> Default ore Position
        oreTransform = eye(3)
        
        %> A cell structure of \c oreCount ore models
        oreModel;
        
        %> zoneSize in meters
        zoneSize = [1,1];        
        
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = robotOres(oreCount,oreTransform)
            if 0 < nargin
                self.oreCount = oreCount;
            end
            
            self.workspaceDimensions = [-self.zoneSize(1)/2, self.zoneSize(1)/2 ...
                                       ,-self.zoneSize(2)/2, self.zoneSize(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of Ores
            for i = 1:self.oreCount
                self.oreModel{i} = self.GetoreModel(['ore',num2str(i)]);
                % Predetermined spawn
                disp(oreTransform)
                CurrentoreTransform = oreTransform{i,1}
                basePose = SE3(SE2(CurrentoreTransform(1,1),CurrentoreTransform(1,2),CurrentoreTransform(1,3)))
                self.oreModel{i}.base = basePose;
                
                 % Plot 3D model
                plot3d(self.oreModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            axis equal
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
        end
        
        function delete(self)
            for index = 1:self.oreCount
                handles = findobj('Tag', self.oreModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end       
        
        %% PlotSingleRandomStep
        % Move each of the Ores forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for oreIndex = 1:self.oreCount
                % Move Forward
                self.oreModel{oreIndex}.base = self.oreModel{oreIndex}.base * SE3(SE2(0.2, 0, 0));
                animate(self.oreModel{oreIndex},0);
                
                % Turn randomly
                % Save base as a temp variable
                tempBase = self.oreModel{oreIndex}.base.T;
                rotBase = tempBase(1:3, 1:3);
                posBase = tempBase(1:3, 4);
                newRotBase = rotBase * rotz((rand-0.5) * 30 * pi/180);
                newBase = [newRotBase posBase ; zeros(1,3) 1];
                           
                % Update base pose
                self.oreModel{oreIndex}.base = newBase;
                animate(self.oreModel{oreIndex},0);                

                % If outside workspace rotate back around
                % Get base as temp
                tempBase = self.oreModel{oreIndex}.base.T;
                
                if tempBase(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < tempBase(1,4) ...
                || tempBase(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < tempBase(2,4)
                    self.oreModel{oreIndex}.base = self.oreModel{oreIndex}.base * SE3(SE2(-0.2, 0, 0)) * SE3(SE2(0, 0, pi));
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    
        
        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function TestPlotManyStep(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 200;
                end
            end
            for i = 1:numSteps
                self.PlotSingleRandomStep();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GetoreModel
        function model = GetoreModel(name)
            if nargin < 1
                name = 'ore';
            end
            [faceData,vertexData] = plyread('HalfSizedRedGreenore.ply','tri');
            link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end