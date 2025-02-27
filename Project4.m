%% Project 4
% Can change the variable under the FK function where we define simplify
% steps
clear all; close all; clc;

%% Initialize Robot
%% Robot Parameters
% Setup
sympref('FloatingPointOutput', true) % This allows floating point coefficient
format short
syms l1 l2 l3 t1 t2 t3 d4 dt1 dt2 dt3 dt4 d1 a2 a3

% Robot Configurations
L1=0.25; L2=0.75; % Configuration 1
%L1=0.5; L2=0.5; % Configuration 2
%L1=0.75; L2=0.25; % Configuration 3

% DH parameters for robot
alpha = [0  0   0]; % Create a symbolic representation of a constant
a   =   [0  L1  L2];
d   =   [0  0   0];
theta = [0  t1  t2];

% Define the robot with Robot Toolbox
L(1) = Link('revolute','d',0,'a',L1,'alpha',0,'modified');
L(2) = Link('revolute','d',0,'a',L2,'alpha',0,'modified');
Robot = SerialLink(L, 'name', '2R SCARA Arm') % Combine Link objects together to form a Robot Object

%% FK Calculations
th = [t1, t2];
FK = Robot.fkine(th)

%% IK Calculations
cos(t1) = (X - 0.2500)/0.7500
sin(t1) = Y/0.7500



%% IK Calculation Function
function [t1, t2, t3, d4] = ScaraIK(Px, Py, Pz, EEorient, d1, a2, a3)
    R = [cos(EEorient), -sin(EEorient),   0;
    sin(EEorient),   cos(EEorient),   0;
    0,              0,              1;]; % calculate rotation matrix

    %TODO: FK and IK equations finish
    %FIXME: % wow 
    d4 = d1 - Pz;

    % Calculate theta2
    ct2 = (Px^2 + Py^2 - a2^2 - a3^2) / (2 * a2 * a3); 
    st2 = sqrt(1 - (ct2)^2);
    t2 = atan2(st2, ct2);

    % Calculate sin(t2) and cos(t2)
    s2 = sin(t2);
    c2 = cos(t2);

    % Calculate theta1
    t1 = atan2(-a3 * s2 * Px + (a2 + a3 * c2) * Py, (a2 + a3 * c2) * Px + a2 * s2 * Py);

    % Calculate theta3
    t3 = atan2(R(2,1), R(1,1)) - t1 - t2;
end