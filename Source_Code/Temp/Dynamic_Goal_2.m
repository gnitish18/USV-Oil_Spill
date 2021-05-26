%% Go to goal using Holonomic Dynamic Model of the system
function [pose, line1] = Dynamic_Goal(goal, pose, line1, t)
    
    % Sampling rate for simulation
    sample_time = 0.01;
    vizRate = rateControl(1/sample_time);
    
    % Tolerance for goal
    goal_radius = 0.1;
    % PID gains
    Kp = [0.1;0.1;0.001]; 
    Kd = [250;250;7.5]; 
    Ki = [0;0;0];
    
    % Mass and Moment of Inertia
    m = 3.5;
    I_zz = 0.03;
    
    % State-Space equation: x_dot = A*x + B*u
    %   Where, x is state vector, u is control variables vector
    %          A is state matrix, B is input matrix
    
    A = [0, 0, 0, cos(pose(3)), -sin(pose(3)), 0;
         0, 0, 0, sin(pose(3)), cos(pose(3)), 0;
         0, 0, 0, 0, 0, 1;
         0, 0, 0, 0, 2*pose(6), 2*pose(5);
         0, 0, 0, -2*pose(6), 0, -2*pose(4);
         0, 0, 0, 0, 0, 0];
    
    B = [0    0    0;
         0    0    0;
         0    0    0;
         cos(pose(3))/m   sin(pose(3))/m   0;
         -sin(pose(3))/m  cos(pose(3))/m   0;
         0    0    1/I_zz;];
    
    % Output equation: y = C*x + D*u
    %   Where, x is state vector, u is control variables vector
    %          C is output matrix, D is direct transmission term (0 here)
    C = [1, 0, 0, 0, 0, 0;
         0, 1, 0, 0, 0, 0;
         0, 0, 1, 0, 0, 0;];
     
    % D = [0 0 0; 0 0 0; 0 0 0];
    
    % Error variables
    E = 0;
    e_old = 0;
    i = 0;
    
    % Desired output state vector
    y_d = [goal; atan2(goal(2) - pose(2), goal(1) - pose(1))];
    % Initial control varible vector (velocities)
    U = [1;1;1];
    
    %figure(3)
    %hold on;
    %cx = axes();
    %axis([0 10000 -10 10])
    %l1 = line(0, 0);
    distance = norm(pose(1:2) - goal(:));
    %[Aa,Bb,Cc,Dd] = c2dm(A,B,C,D,sample_time); 
    %norm(U)

    % Iterate until USV is within the tolerance
    while distance > goal_radius
        i = i + 1;
        %figure(3)
        %l1.XData = [l1.XData i];
        %l1.YData = [l1.YData pose(3)];
        y_d(3) = atan2(goal(2) - pose(2), goal(1) - pose(1));
        distance = norm(pose(1:2) - goal(:));
        
        % Computing the errors in state and updation
        e = y_d - C*pose;
        e(3) = atan2(sin(e(3)), cos(e(3)));
        e_dot = e - e_old;
        E = E + e;
        U = Kp.*e + Kd.*e_dot + Ki.*E;
        %U(1) = U(1)*cos(pose(3));
        %U(2) = U(2)*sin(pose(3));
        
        % Update errors
        e_old = e;
        
        % Update the state variables
        %pose = (Aa*pose + Bb*U);%*sample_time;
        
        pose = pose + (A*pose + B*U)*sample_time;
%         pose = pose + ([pose(4)*cos(pose(3))-pose(5)*sin(pose(3));
%                         pose(3)*sin(pose(3))+pose(5)*cos(pose(3)); 
%                         pose(6);
%                         pose(5)*pose(6); 
%                         -1*pose(4)*pose(6);
%                         0] + [0;0;0;1/m;1/m;1/I_zz].*[0;0;0;U])*sample_time;

        % Plot the live location of USV
        line1.XData = [line1.XData pose(1,1)];
        line1.YData = [line1.YData pose(2,1)];
        
        % Simulation
        Simulation(pose, t);
        
        % Sampling Rate
        waitfor(vizRate);
    end

end