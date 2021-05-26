%% Go to goal using Holonomic Dynamic Model of the system
function [pose] = Controller(goal, pose, t)
    
    % Sampling rate for simulation
    sample_time = 0.03;
    sim_time = 35;
    vizRate = rateControl(1/sample_time);
    timestep = 0:sample_time:sim_time;
    
    % Limits of thrusters
    Max_Thrust = 20;
    Max_Moment = 3;
    
    % PID gains
    Kp = [0.1;0.1;0.01];
    Kd = [500;500;5];
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
    
    % Lists for plotting data
    x_list = zeros(fix(sim_time/sample_time),1);
    y_list = zeros(fix(sim_time/sample_time),1);
    x_e_list = zeros(fix(sim_time/sample_time),1);
    y_e_list = zeros(fix(sim_time/sample_time),1);
    psi_e_list = zeros(fix(sim_time/sample_time),1);
    u_list = zeros(fix(sim_time/sample_time),1);
    v_list = zeros(fix(sim_time/sample_time),1);
    r_list = zeros(fix(sim_time/sample_time),1);
    Fx_list = zeros(fix(sim_time/sample_time),1);
    Fy_list = zeros(fix(sim_time/sample_time),1);
    Mz_list = zeros(fix(sim_time/sample_time),1);
    
    % Plot of desired trajectory
    figure(2)
    plot(goal(:,1), goal(:,2),':r','LineWidth',2);
    axis([-5 20 -10 10])
    hold on;

    % Iterate for the given simulation time
    for time = 1:length(timestep)
        
        % Update goal points
        if time < length(goal)
            y_d = [goal(time,1); 
                   goal(time,2); 
                   atan2(goal(time,2) - pose(2), goal(time,1) - pose(1))];
        end
        
        % Computing the errors in state and updation
        e = y_d - C*pose;
        e(3) = atan2(sin(e(3)), cos(e(3)));
        e_dot = e - e_old;
        E = E + e;
        U = Kp.*e + Kd.*e_dot + Ki.*E;
        
        % Limiting the Thrusts and Moment
        if U(1) > Max_Thrust
            U(1) = Max_Thrust;
        elseif U(1) < -Max_Thrust
            U(1) = -Max_Thrust;
        end
        if U(2) > Max_Thrust
            U(2) = Max_Thrust;
        elseif U(2) < -Max_Thrust
            U(2) = -Max_Thrust;
        end
        if U(3) > Max_Moment
            U(3) = Max_Moment;
        elseif U(3) < -Max_Moment
            U(3) = -Max_Moment;
        end
        
        % Update errors
        e_old = e;
        
        % Update the state variables
        pose = pose + (A*pose + B*U)*sample_time;

        % Append the updated data to lists
        x_list(time) = pose(1);
        y_list(time) = pose(2);
        x_e_list(time) = e(1);
        y_e_list(time) = e(2);
        psi_e_list(time) = e(3);
        u_list(time) = pose(4);
        v_list(time) = pose(5);
        r_list(time) = pose(6);
        Fx_list(time) = U(1);
        Fy_list(time) = U(2);
        Mz_list(time) = U(3);
        
        % Plotting the live location of USV
        figure(2)
        plot(x_list(1:time),y_list(1:time),'-b')
        hold on

        % Simulation
        Simulation(pose, t);
        
        % Sampling Rate
        waitfor(vizRate);
    end
    
    % Trajectory Plot
    figure(2)
    legend('Desired Trajectory','Actual Trajectory')
    title('Trajectory Plot')
    xlabel('x-coordinate (m)')
    ylabel('y-coordinate (m)')
    
    % Errors plot
    figure(3)
    title('Errors vs Time')
    xlabel('Time (sec)')
    ylabel('Distance (m) / Angle (rad)')
    plot(timestep(1:time),x_e_list(1:time),'-b')
    hold on
    plot(timestep(1:time),y_e_list(1:time),'-r')
    hold on
    plot(timestep(1:time),psi_e_list(1:time),'-g')
    hold on
    legend('Error in x','Error in y','Error in psi')
    grid on
    title('Errors vs Time')
    xlabel('Time (sec)')
    ylabel('Distance (m) / Angle (rad)')
    
    % Velocities plot
    figure(4)
    title('Velocities vs Time')
    xlabel('Time (sec)')
    ylabel('Linear Velocity (m/s) / Angular Velocity (rad/s)')
    plot(timestep(1:time),u_list(1:time),'-b')
    hold on
    plot(timestep(1:time),v_list(1:time),'-r')
    hold on
    plot(timestep(1:time),r_list(1:time),'-g')
    hold on
    legend('u','v','R')
    grid on
    title('Velocities vs Time')
    xlabel('Time (sec)')
    ylabel('Linear Velocity (m/s) / Angular Velocity (rad/s)')
    
    % Control Inputs plot
    figure(5)
    title('Control Inputs vs Time')
    xlabel('Time (sec)')
    ylabel('Force (N) / Moment (Nm)')
    plot(timestep(1:time),Fx_list(1:time),'-b')
    hold on
    plot(timestep(1:time),Fy_list(1:time),'-r')
    hold on
    plot(timestep(1:time),Mz_list(1:time),'-g')
    hold on
    legend('Fx','Fy','Mz')
    grid on
    title('Control Inputs vs Time')
    xlabel('Time (sec)')
    ylabel('Force (N) / Moment (Nm)')

    % End of Simulation
    disp('Simulation Complete')
    
end