%% PID Control of an Unmanned Surface Vehicle (USV) using a dynamic model
clear; close all; clc;

sim_time = 100;
sampling = 0.1;

%% Set-up Simulation Model

figure(1)
ax = axes('XLim',[-10 10],'YLim',[-5 20],'ZLim',[-2.5 2.5]);
view(3);    % View all the axes (3D)
grid on;

% Modelling water
Water_x = -10:0.5:10;
Water_y = -10:0.5:20;
Water_z = -1.5:0.1:1;
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

%% Trajectories, Control and Guidance of USV

% Trajectory selection
disp('Enter trajectory:');
traj = input(' 1. Circle \n 2. Square \n 3. Sine \n');
switch traj
    % Circle
    case 1
        pose = [0;0;-pi/2;0;0;0];
        n = sim_time/sampling; R = 8;
        k = 2*pi*linspace(0,1,n)' - 3*pi*ones(n,1);
        x = R*ones(n,1) + R.*cos(k);
        y = zeros(n,1) + R.*sin(k);
    % Square
    case 2
        pose = [0;0;-pi/2;0;0;0];
        n = sim_time/sampling; n1 = fix(n/4);
        x0 = zeros((n1/2),1);
        y0 = linspace(0,-8,fix(n1/2))';
        x1 = linspace(0,16,n1)';
        y1 = -8*ones(n1,1);
        x2 = 16*ones(n1,1);
        y2 = linspace(-8,8,n1)';
        x3 = linspace(16,0,n1)';
        y3 = 8*ones(n1,1);
        x4 = zeros((n1/2),1);
        y4 = linspace(8,0,fix(n1/2))';
        x = [x0;x1;x2;x3;x4];
        y = [y0;y1;y2;y3;y4];
    % Sine
    case 3
        pose = [0;0;pi/3;0;0;0];
        n = sim_time/sampling;
        x = 4*pi*linspace(0,1,n)';
        y = 5*sin(x);
end

goal = [x, y];
[pose] = Controller(goal, pose, t);
