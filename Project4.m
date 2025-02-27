%% Project 4
% Can change the variable under the FK function where we define simplify
% steps
clear all; close all; clc;

%% Initialize Robot
%% Robot Parameters
% Setup
sympref('FloatingPointOutput', true) % This allows floating point coefficient
format short
syms L1 L2 t1 t2

% Robot Configurations
%L1=0.25; L2=0.75; % Configuration 1
%L1=0.5; L2=0.5; % Configuration 2
%L1=0.75; L2=0.25; % Configuration 3

% DH parameters for robot
alpha = [ 0 0   0]; % Create a symbolic representation of a constant
a   =   [0 L1  L2];
d   =   [0   0 0];
theta = [t1  t2 0];

% Define the robot with Robot Toolbox
L(1) = Link('revolute','d',0,'a',L1,'alpha',0,'modified');
L(2) = Link('revolute','d',0,'a',L2,'alpha',0,'modified');
Robot = SerialLink(L, 'name', '2R SCARA Arm') % Combine Link objects together to form a Robot Object

%% FK Calculations
[T, Tfinal] = FK(alpha, a, d, theta);
%% IK Calculations
%cos(t1) = (X - 0.2500)/0.7500
%sin(t1) = Y/0.7500

x = L2*cos(t1+t2)+L1*cos(t1);
y = L2*sin(t1+t2)+L1*sin(t1);

simplify(sqrt(x^2+y^2), 'Steps', 10)

%% IK Calculation Function
function [t1, t2] = IK(Px, Py, Pz, L1, L2) % All equations were calculated with inverse kinematics
    % Calculate theta2
    ct2 = ((Px^2 + Py^2)^2 - L1^2 - L2^2) / (2 * L1 * L2); 
    st2 = sqrt(1 - (ct2)^2);
    t2 = atan2(st2, ct2);

    % Calculate sin(t2) and cos(t2)
    s2 = sin(t2);
    c2 = cos(t2);

    % Calculate theta1
    st1 = -L2 * s2 * Px + (L1 + L2 * c2) * Py; %denominator cancels out
    ct1 = (L1 + L2 * c2) * Px + L2 * s2 * Py;
    t1 = atan2(st1,ct1);
end

%%
function [T, Tfinal] = FK(alpha, a, d, theta) % Calculated FK (263A Final Exam code)
    %syms d1 a2 a3
    % Initialization of the transformation from the base to the end effector
    T0_ee = eye(4);
    T = struct(); % Initialize structure array for T
    
    for i = 1:length(alpha)
        fprintf('T (%d) to (%d)\n', i-1, i)
        % Calculate each transformation matrix and store in structure array
        T(i).matrix = TF(a(i), alpha(i), d(i), theta(i)); % Assuming TF is defined elsewhere
        pretty(simplify(T(i).matrix))
        T0_ee = T0_ee * T(i).matrix; % Sequential multiplications
    end
    
    % Simplify the final transformation
    Tfinal = simplify(T0_ee, 'Steps', 10);
    
    % Display and return Tfinal
    pretty(Tfinal);
end

function T = TF(a,alpha,d,theta) % This function is used in FK
T = [cos(theta) -sin(theta) 0 a
sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d
0 0 0 1];
end