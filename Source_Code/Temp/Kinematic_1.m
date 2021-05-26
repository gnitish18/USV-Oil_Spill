clear; close all; clc;
%Kinematics - Transformation

% <>_a - Interial Frame : <>_b - Body Frame

%Velocity components in X, Y and Z axis
%-----------------------------------------------
u_a = 5;            %INPUTS
v_a = 2;            %Random values given for testing
w_a = 3;
%-----------------------------------------------

u_b = 0;
v_b = 0;
w_b = 0;

%Velocity Vectors
P_a = [u_a; v_a; w_a];
P_b = [u_b; v_b; w_b];

%Rotation angles 
psi = 30;           %Rotation about z-axis
theta = 45;         %Rotation about y-axis
phi = 60;           %Rotation about x-axis

%Rotation matrix about X -> Y -> Z
R_ab = rotx(phi)*roty(theta)*rotz(psi)
P_b = R_ab*P_a; 

%-------------------------------------------------------------------------------
%Angular Transformation

phi_dot = 0;        %Angular Velocity in X 
theta_dot = 0;      % Y
psi_dot = 0;        % Z

%-----------------------------------------------
p = 6;              % INPUTS
q = 2;              % Random values given for testing
r = 3;
%-----------------------------------------------

eta_dot = [phi_dot; theta_dot; psi_dot];    %Angular vel vector in inertial frame
w_b = [p; q; r];                            %Angular velocity vector in body frame

%Transformation matrix for angular rotation
Rt_ab = [ 1,     sin(phi)*tan(theta),        cos(phi)*tan(theta);
          0,         cos(phi),                   -sin(phi);
          0,     sin(phi)/cos(theta),         cos(phi)/cos(theta)]
      
eta_dot = Rt_ab*w_b;
