
classdef ThreeFingerGripper < handle
    properties
        model;
        finger1Joint;
        finger2Joint;
        finger3Joint;
    end
    
    methods
        function self = ThreeFingerGripper()

            % Define D&H for gripper fingers.
            L1 = Link('d', 0, 'a', 0.1, 'alpha', 0, 'qlim', [-pi pi]);
            L2 = Link('d', 0, 'a', 0.1, 'alpha', 0, 'qlim', [-pi pi]);
            L3 = Link('d', 0, 'a', 0.1, 'alpha', 0, 'qlim', [-pi pi]);

            self.finger1Joint = SerialLink([L1 L2 L3],'name','finger1');

            self.finger2Joint = SerialLink([L1 L2 L3],'name','finger2');

            self.finger3Joint = SerialLink([L1 L2 L3],'name','finger3');

            self.model = {self.finger1Joint, self.finger2Joint, self.finger3Joint};
            
        end

        function plot(self, q)

           self.finger1Joint.plot(q(1,:));

           self.finger2Joint.base = transl(0.1, 0, -0.07);
           self.finger2Joint.plot(q(2,:));

           self.finger3Joint.base = transl(0.1, 0, 0.07);
           self.finger3Joint.plot(q(2,:));

        end 

        function animate(self, q1, q2)

            steps = 100;
            qPath = jtraj(q1,q2,steps/2);

            for i = 1:steps

                self.finger1Joint.animate(qPath(i,[1 4 7]));

                self.finger2Joint.base = transl(0.1, 0, -0.07);
                self.finger2Joint.animate(qPath(i,[2 5 8]));

                self.finger3Joint.base = transl(0.1, 0, 0.07);
                self.finger3Joint.animate(qPath(i,[3 6 9]));

                drawnow();

            end
        end
    end
end