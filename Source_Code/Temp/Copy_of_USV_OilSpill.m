%% PID Control of an Unmanned Surface Vehicle (USV) using a dynamic model
clear; close all; clc;

% Initial robot configuration - x, y, theta, u, v, omega 
pose_kin = [0;0;0];
pose_dyn = [0;0;0;0;0;0];

sim_time = 200;
sample_time = 0.01;
vizRate = rateControl(1/sample_time);

%% Set-up Simulation Model
 
figure(1)
ax = axes('XLim',[-10 10],'YLim',[-10 10],'ZLim',[-2.5 2.5]);
view(3);    % view all the axes (3D)
grid on;
 
% Modelling water
Water_x=-10:0.5:10;
Water_y=-10:0.5:10;
Water_z=-1.5:0.1:1;
[Water_X,Water_Y,Water_Z] = meshgrid(Water_x,Water_y,Water_z);
I = Water_Z <= 0;
scatter3(Water_X(I),Water_Y(I),Water_Z(I),'.');

% Light source
light('Position',[-10 -10 2],'Style','infinite')
light('Position',[10 10 2.5],'Style','infinite')

% Robot (USV) Model
[con_x,con_y,con_z] = cylinder([0.1 0.0],12);
[cyl_x,cyl_y,cyl_z] = cylinder([0.2 0.2],12);
% Surfaces of USV
h(1) = surface(4*con_x,0.5*con_z,-0.3*con_y,'FaceColor','y');
h(2) = surface(2*cyl_x,-1*cyl_z,0.15*cyl_y,'FaceColor','y');
h(3) = surface(4*con_x,(0.01*con_z)-1,-0.3*con_y,'FaceColor','y');
h(4) = surface(0.1,1,0.1,'FaceColor','w');
h(1).EdgeColor = 'none'; h(1).FaceLighting = 'gouraud';
h(2).EdgeColor = 'none'; h(2).FaceLighting = 'gouraud';
h(3).EdgeColor = 'none'; h(3).FaceLighting = 'gouraud';
% Transforming the surfaces wrt robot position
t = hgtransform('Parent',ax);
set(h,'Parent',t)
set(gcf,'Renderer','opengl')
drawnow

%% Path-tracking and set-points of USV

figure(2)
bx = axes();
hold on;
lin = line(pose_kin(1,1), pose_kin(2,1));
axis([-10 10 -10 10])

%% Trajectories, Control and Guidance of USV

% Get the desired model type
mod = input('Enter model type: \n 1. Kinematic \n 2. Dynamic \n');

% Get the desired mode of traversal
disp('Enter traversal mode:');
trav_mode = input(' 1. Waypoints \n 2. Trajectory \n');

switch trav_mode
    
    % Waypoints
    case 1
        n = 5; 
        % Generate random waypoints
        x = 15*rand(n,1) - 7.5;
        y = 15*rand(n,1) - 7.5;
        % Go to goal for every waypoint
        for i = 1:length(x)
            goal = [x(i); y(i)];
            figure(2)
            plot(x(i), y(i), 'sr');
            if mod == 1
                [pose_kin, lin] = Kinematic_Goal(goal, pose_kin, lin, t);
            elseif mod == 2
                [pose_dyn, lin] = Dynamic_Goal(goal, pose_dyn, lin, t);
            end
            figure(2)
            plot(x(i), y(i), 'sg');
        end
        
    % Trajectory
    case 2
        disp('Enter trajectory:');
        traj = input(' 1. Circle \n 2. Square \n 3. Sine \n');
        switch traj
            % Circle
            case 1
                n = sim_time/sample_time; R = 8;
                k = 2*pi*linspace(0,1,n)';
                x = zeros(n,1) + R.*cos(k);
                y = zeros(n,1) + R.*sin(k);
            % Square
            case 2
                n = sim_time/sample_time; n1 = fix(n/4);
                x1 = linspace(-8,8,n1)';
                y1 = -8*ones(n1,1);
                x2 = 8*ones(n1,1);
                y2 = linspace(-8,8,n1)';
                x3 = linspace(8,-8,n1)';
                y3 = 8*ones(n1,1);
                x4 = -8*ones(n1,1);
                y4 = linspace(8,-8,n1)';
                x = [x1;x2;x3;x4];
                y = [y1;y2;y3;y4];
            % Sine
            case 3
                n = sim_time/sample_time;
                k = 8*pi*linspace(0,1,n)' - 4*pi;
                x = 0.5*k;
                y = 5*sin(k);
        end
        % plotting the desired trajectory
        plot(x, y, ':r');
        
        % Go to goal for every waypoint
        for i = 1:length(x)
            goal = [x(i); y(i)];
            if mod == 1
                [pose_kin, lin] = Kinematic_Goal(goal, pose_kin, lin, t);
            elseif mod == 2
                [pose_dyn, lin] = Dynamic_Goal(goal, pose_dyn, lin, t);
            end
        end
        
    otherwise
        disp('Enter a valid choice');
end    
