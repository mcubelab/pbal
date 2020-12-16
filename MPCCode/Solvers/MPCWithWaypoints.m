classdef MPCWithWaypoints
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        tht_min             % minimum distance between waypoints
        mpc_tv              % time-varying MPC object
        tht_goal            % goal orientation
        omega_desired       % the angular velocity to linearize about
    end
    
    methods
        function obj = MPCWithWaypoints(waypoint_params, ...
                system_to_control, mpc_params)
            
            obj.mpc_tv = TimeVaryingMPC2(system_to_control, mpc_params);
            obj.tht_min = waypoint_params.tht_min;
            obj.omega_desired = waypoint_params.omega_desired;
            obj.tht_goal = mpc_params.x0(3);  % third entry in x0 should be theta
            
        end
        
        function [Xpredicted, Upredicted, exitflag] = run_mpc_nearest_waypoint(obj, ...
                xk, update_linearization_flag)
            
            % want a new waypoint
            if update_linearization_flag
                
                % build waypoints spaced at least tht_min apart to the goal
                % unless you are at the goal
                if xk(3) <= (obj.tht_goal - obj.tht_min)
                    %                     disp('less than goal')
                    waypoint = xk(3) + obj.tht_min;
                    omega = obj.omega_desired;
                elseif xk(3) > (obj.tht_goal + obj.tht_min)
                    %                     disp('greater than goal')
                    waypoint = xk(3) - obj.tht_min;
                    omega = -obj.omega_desired;
                else
                    %                     disp('near goal')
                    waypoint = obj.tht_goal;
                    omega = 0 * obj.omega_desired;
                end
                
                % update goal state
                obj.mpc_tv.x0(3) = waypoint;
                obj.mpc_tv.x0(6) = omega;
                
                
            end
            
%             disp('current angle:')
%             disp(rad2deg(xk(3)))
%             
%             disp('next waypoint:')
%             disp(rad2deg(waypoint))
%             
%             disp(rad2deg(obj.mpc_tv.x0(3)))
%             disp(rad2deg(obj.mpc_tv.x0(6)))
            
            % run mpc
            [Xpredicted, Upredicted, exitflag] = ...
                obj.mpc_tv.run_mpc(xk, update_linearization_flag);
        end
    end
end

