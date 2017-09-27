classdef Target < simiam.ui.Drawable
    
% Modification added to the Sim.I.am robot simulator provided by GRITS Lab of Georgia Tech
% Author of Mod: Marcus Futterlieb
    
    properties
       type
       visibility
    end
    
    methods
        function obj = Target(parent, pose)
            obj = obj@simiam.ui.Drawable(parent, pose);
            obj.type = 'target';
            geometry = [ 0.07, 0.07,  1;...
                        -0.07, 0.07,  1;...
                         0,  -0.07,  1];
            surface = obj.add_surface(geometry, [0.0 0.0 0.8]); %rgb color of target surface
            set(surface.handle_, 'EdgeColor', 'r');
            
            
            obj.visibility = true;
        end
    end
    
end

