classdef Obstacle_dyn < simiam.ui.Drawable
    
% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software
    
    properties
       type
       radius
       movement_type
       direction
       speed
       circle_center_x
       circle_center_y
       circle_old_x
       circle_old_y
       circle_full_rot_distance
       bf_a
       bf_b
       bf_trigger
       bf_forward
       bf_back_direction
    end
    
    methods
        function obj = Obstacle_dyn(parent, pose, geometry, direction, speed, radius, movement_type)
            obj = obj@simiam.ui.Drawable(parent, pose);
            obj.type = 'obstacle_dyn';
            geometry(:,3) = ones(size(geometry,1),1);
            surface = obj.add_surface(geometry, [1 0.4 0.4]); %rgb color of obstacle surface
            set(surface.handle_, 'EdgeColor', 'r');
            
            % WrapToPi Wrap angle in radians to [-pi pi]
            while ( direction > pi || direction < -pi)
                if direction > pi
                    direction = direction - 2*pi ;
                elseif direction < -pi
                    direction = 2*pi + direction;
                end
            end 
            
            obj.direction       = direction;
            
            obj.speed           = speed;
            obj.radius          = radius;
            obj.movement_type   = char(movement_type);
            if (  ((strcmp(obj.movement_type,'back_forth')   ==true) + ...
                  (strcmp(obj.movement_type,'straight_line')==true)  + ...
                  (strcmp(obj.movement_type,'circle')       ==true)) < 1 )
                error('unknown movmen type of dynmaic obstacle');
            else
                disp('dynamic obstacle recognized');
            end%if
            
            if ( (strcmp(obj.movement_type,'circle'))  ==true )
                [x,y,theta]                     =  pose.unpack();
                obj.circle_old_x                = x;
                obj.circle_old_y                = y;
                obj.circle_center_x             = x-obj.radius;
                obj.circle_center_y             = y;
                obj.circle_full_rot_distance    = 2*pi*obj.radius;
            end%if
            
            if ( (strcmp(obj.movement_type,'back_forth'))  ==true )
                [x,y,theta]                     =  pose.unpack();
                obj.bf_a                        = [x;y];
                obj.bf_b                        = [x+((cos(obj.direction))*obj.radius);y+((sin(obj.direction))*obj.radius)];
%                 disp(obj.bf_a)
%                 disp(obj.bf_b)
                obj.bf_trigger                  = 0.1;
                obj.bf_forward                  = true;
                if (obj.direction>=0)
                    obj.bf_back_direction       = obj.direction - pi;
                else
                    obj.bf_back_direction       = obj.direction + pi;
                end%if
            end%if
            
        end
        
        function pose = update_state(obj, pose, dt)
            [x_old,y_old,theta_old] =  pose.unpack();
            
            if      (strcmp(obj.movement_type,'straight_line')==true)
                %% straight line
                x_new       = x_old + (((cos(obj.direction))*obj.speed)*dt);
                y_new       = y_old + (((sin(obj.direction))*obj.speed)*dt);
                theta_new   = theta_old;
            elseif  (strcmp(obj.movement_type,'back_forth')==true)
                %% back forth
                
                if (obj.bf_forward == true)
                    x_new       = x_old + (((cos(obj.direction))*obj.speed)*dt);
                    y_new       = y_old + (((sin(obj.direction))*obj.speed)*dt);
                    theta_new   = obj.direction;
                    if (  norm(obj.bf_b-[x_old;y_old])<obj.bf_trigger  )
                        obj.bf_forward = false;
                    else
                        obj.bf_forward = true;
                    end%if
                else
                    x_new       = x_old + (((cos(obj.bf_back_direction))*obj.speed)*dt);
                    y_new       = y_old + (((sin(obj.bf_back_direction))*obj.speed)*dt);
                    theta_new   = obj.bf_back_direction;
                    if (  norm(obj.bf_a-[x_old;y_old])<obj.bf_trigger  )
                        obj.bf_forward = true;
                    else
                        obj.bf_forward = false;
                    end%if
                    
                end%if
                
                
                
                
            elseif  (strcmp(obj.movement_type,'circle')       ==true)
                %% circle
                distance_move       = obj.speed * dt;
                angular_move        = distance_move/obj.circle_full_rot_distance;
                vec_center_current  = [x_old-obj.circle_center_x; y_old-obj.circle_center_y];
                
                angle_current       = atan2(vec_center_current(2),vec_center_current(1));
                
                % WrapToPi Wrap angle in radians to [-pi pi]
                while ( angle_current > pi || angle_current < -pi)
                    if angle_current > pi
                        angle_current = angle_current - 2*pi ;
                    elseif angle_current < -pi
                        angle_current = 2*pi + angle_current;
                    end
                end 
                
                
                angle_next          = angle_current + angular_move;
                
                x_new               = obj.circle_center_x + cos(angle_next) * obj.radius;
                y_new               = obj.circle_center_y + sin(angle_next) * obj.radius;
                
                %calc theta
                heading_tmp         = [x_new - obj.circle_old_x;y_new - obj.circle_old_y];
                theta_new_tmp       = atan2(heading_tmp(2),heading_tmp(1));
                theta_new           = obj.mfu_wrap_to_pi(theta_new_tmp);
                %save latest position
                obj.circle_old_x    = x_new;
                obj.circle_old_y    = y_new;
            else
                error('unknown movmentype of dynmaic obstacle');
            end%if
            
            %pose = [x_new, y_new, theta_new];
            pose.set_pose([x_new, y_new, theta_new]);
            obj.update_pose(pose);
        end
        
        %% Wrap to pi
        function angle              = mfu_wrap_to_pi(obj,angle)
            while ( abs(angle) > pi)
                if (angle > pi)
                    angle = angle - 2*pi ;
                elseif (angle < -pi)
                    angle = 2*pi + angle;
                else
                    ('angle was in range -- no need for wrapping');
                end
            end 
        end%function

        
    end
    
end

