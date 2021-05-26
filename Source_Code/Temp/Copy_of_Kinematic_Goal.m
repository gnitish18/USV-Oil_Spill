%% Go to goal using Unicyclic Kinematic Model of the system
function [pose, line1] = Kinematic_Goal(goal, pose, line1, t)

    % Sampling rate for simulation
    sample_time = 0.01;
    vizRate = rateControl(1/sample_time);
    
    % Tolerance for goal
    goal_radius = 0.1;
    distance = norm(pose(1:2) - goal(:));
    % PID gains for linear and angular velocities
    Kp_w = 5; Kd_w = 1; Ki_w = 0;
    Kp_v = 0.5; Kd_v = 0; Ki_v = 0; C = 0.5;

    % Error variables
    W_E = 0; V_E = 0;
    W_e_old = 0; V_e_old = 0;

    % Iterate until USV is within the tolerance
    while distance > goal_radius
        
        % Required angular turn
        theta_d = atan2(goal(2,1) - pose(2,1), goal(1,1) - pose(1,1));
        
        % Computing the errors in body frame angular velocity and updation
        error = theta_d - pose(3,1);
        W_e = atan2(sin(error), cos(error));
        W_e_dot = W_e - W_e_old;
        W_E = W_E + W_e;
        w = Kp_w*W_e + Kd_w*W_e_dot + Ki_w*W_E;
        
        % Computing the errors in body frame velocity and updation
        distance = norm(pose(1:2) - goal(:));
        V_e = distance;
        V_e_dot = V_e - V_e_old;
        V_E = V_E + V_e;
        v0 = C + Kp_v*V_e + Kd_v*V_e_dot + Ki_v*V_E;
        
        % Velocities in inertial frame
        x_dot = v0*cos(pose(3,1));
        y_dot = v0*sin(pose(3,1));
        vel = [x_dot; y_dot; w];
        
        % Update errors
        W_e_old = W_e;
        V_e_old = V_e;
        distance = norm(pose(1:2) - goal(:));
        
        % Update the state variables
        pose = pose + vel*sample_time;
        
        % Plot the live location of USV
        line1.XData = [line1.XData pose(1,1)];
        line1.YData = [line1.YData pose(2,1)];
        
        % Simulation
        Simulation(pose, t);
        
        % Sampling Rate
        waitfor(vizRate);
    end

end