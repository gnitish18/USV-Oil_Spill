%% Go to goal using Holonomic Dynamic Model of the system
function [pose, line1] = Dynamic_Goal(goal, pose, line1, t)
    
    % Sampling rate for simulation
    sample_time = 0.01;
    vizRate = rateControl(1/sample_time);
    
    % Tolerance for goal
    goal_tolerance = 0.1;
    % PID gains
    Kp = [0.1;0.1;0.001]; 
    Kd = [500;500;7.5];
    Ki = [0;0;0];
    
    % Mass and Moment of Inertia
    m = 1.8;
    I_zz = 0.03;
    
    % State-Space equation: x_dot = A*x + B*u
    %   Where, x is state vector, u is control variables vector
    %          A is state matrix, B is input matrix
    
    A = [0, 0, 0, 1, 0, 0;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 1;
         0, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0];
    
    B = [0    0    0;
         0    0    0;
         0    0    0;
         1/m  0    0;
         0    1/m  0;
         0    0    1/I_zz;];
    
    % Output equation: y = C*x + D*u
    %   Where, x is state vector, u is control variables vector
    %          C is output matrix, D is direct transmission term (0 here)
    C = [1, 0, 0, 0, 0, 0;
         0, 1, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;];
    
    % Error variables
    E = 0;
    e_old = 0;
    
    % Desired output state vector
    y_d = [goal; atan2(goal(2) - pose(2), goal(1) - pose(1))];
    % Initial control varible vector (velocities)
    U = [1;1;1];
    
    e = y_d - C*pose;

    % Iterate until USV is within the tolerance
    for timestep = 0:0.01:100
        i = fix(timestep); 
        % Computing the errors in state and updation
        e = y_d - C*pose;
        e(3) = atan2(sin(e(3)), cos(e(3)));
        e_dot = e - e_old;
        E = E + e;
        U = Kp.*e + Kd.*e_dot + Ki.*E;
        U(2) = 0;
        %if U(1) > 1000
        %    U(1) = 1000;
        %elseif U(1) < -1000
        %    U(1) = -1000;
        %end
        %if U(2) > 1000
        %    U(2) = 1000;
        %elseif U(2) < -1000
        %    U(2) = -1000;
        %end
        
        % Update errors
        e_old = e;
        
        % Update the state variables
        pose = pose + (A*pose + B*U)*sample_time;

        % Plot the live location of USV
        line1.XData = [line1.XData pose(1,1)];
        line1.YData = [line1.YData pose(2,1)];
        
        % Simulation
        Simulation(pose, t);
        
        % Sampling Rate
        waitfor(vizRate);
    end
    disp('out of loop')
end