%% Project 3
% Can change the variable under the FK function where we define simplify
% steps
clear all; close all; clc
%% Velocity Propogation
%% Robot Parameters
% Setup
sympref('FloatingPointOutput',true) % This allows floating point coefficient
format short
syms l1 l2 l3 t1 t2 t3 d4 dt1 dt2 dt3 dt4

% Given Robot Link Parameters
d1 = 0.416; % m
a2 = 0.325; % m
a3 = 0.225; % m

% DH parameters for robot
alpha = [0  0   0   sym(pi)]; % Create a symbolic representation of a constant
a   =   [0  a2  a3  0];
d   =   [d1 0   0   d4];
theta = [t1 t2  t3  0];

% Define the robot with Robot Toolbox
L(1) = Link('revolute','d',d1,'a',0,'alpha',0,'modified');
L(2) = Link('revolute','d',0,'a',a2,'alpha',0,'modified');
L(3) = Link('revolute','d',0,'a',a3,'alpha',0,'modified');
L(4) = Link('prismatic','a',0,'alpha',pi,'theta', 0,'modified', 'qlim', [0 d1]); % add limits of the d4
Robot = SerialLink(L, 'name', 'SCARA Mitsubishi Arm') % Combine Link objects together to form a Robot Object
%% FK Forward Kinematics
% Calculate the between T and final T matrix
[T, Tfinal] = FK(alpha, a, d, theta);

% Get T matrices from T structure
T01 = T(1).matrix;
T12 = T(2).matrix;
T23 = T(3).matrix;
T34 = T(4).matrix;
T04 = Tfinal;

% Extract the rotation matrix and 
% translation vector from the transofrmation matrix
[R01, P01] = tr2rt(T01); R10 = transpose(R01);
[R12, P12] = tr2rt(T12); R21 = transpose(R12);
[R23, P23] = tr2rt(T23); R32 = transpose(R23);
[R34, P34] = tr2rt(T34); R43 = transpose(R34);
[R04, P04] = tr2rt(T04);
%% Calculate Angular Velocity
% All revolute joints: 
w0 = [0;0;0];
w1 = [0;0;dt1];
w2 = simplify(R21*w1 + [0;0;dt2]);
w3 = simplify(R32*w2 + [0;0;dt3]);
w4 = simplify(R43*w3);
%% Calculate Linear Veolcity
v0 = [0;0;0];
% Collect function helps you to collect the coefficent of
% the expression you specified. 
v1 = collect(R10*(cross(w0,P01)+v0),[dt1 dt2 dt3 dt4]); % This functions grabs the coefficient values for the dt1-dt4
v2 = collect(R21*(cross(w1,P12)+v1),[dt1 dt2 dt3 dt4]);
v3 = collect(R32*(cross(w2,P23)+v2),[dt1 dt2 dt3 dt4]);
v4 = collect(R43*(cross(w3,P34)+v3),[dt1 dt2 dt3 dt4]);
v4 = simplify(v4);
% Derive our Jacobian Matrix
[JV4] = equationsToMatrix([v4],[dt1 dt2 dt3]); % Make matrix form times the dt1-dt3 to get v4
[JW4] = equationsToMatrix([w4],[dt1 dt2 dt3]);
J4_VP = simplify([JV4;JW4]);
% We are not done yet!!
% Need to transform back to base frmae
% Define Jacobain Rotation Matrix:
JT = [R04, zeros(3,3);zeros(3,3), R04];
% Derive Jacobian Matrix in Base Frame:
J0_VP = simplify(JT*J4_VP)
%% Lets plot the velocity ellipsoid
close all; 
qn = [0 pi/2 pi/2 0];
Robot.plot(qn, 'workspace', [-1,1,-1,1,-1,1]);
hold on
Robot.vellipse(qn);

%% Explicit Method %%
%% Forward Kinematics

T03 = simplify(T01 * T12 * T23);
T02 = simplify(T01 * T12);
P02 = T02(1:3, 4);
P03 = T03(1:3, 4);

% syms t1 t2 t3 t4 t5 t6 a2 a3 d3 d4 
% assume([t1 t2 t3 t4 t5 t6 a2 a3 d3 d4],'real')
% 
% % Using robotic toolbox to generate forward Kinematic will 
% % Intruoduce many very small coefficients that cannot be cancelled to 0
% % Lets use Matlab symbolic toolbox to eliminate those coefficients
% T0_1 = [cos(t1) -sin(t1) 0 0;
%         sin(t1) cos(t1) 0 0;
%         0 0 1 0;
%         0 0 0 1];
% T1_2 = [cos(t2) -sin(t2) 0 0;
%         0 0 1 0;
%        -sin(t2) -cos(t2) 0 0;
%         0 0 0 1];
% T2_3 = [cos(t3) -sin(t3) 0 a2;
%         sin(t3) cos(t3) 0 0;
%         0 0 1 d3;
%         0 0 0 1];
% T3_4 = [cos(t4) -sin(t4) 0 a3;
%         0 0 1 d4;
%        -sin(t4) -cos(t4) 0 0;
%         0 0 0 1];
% T4_5 = [cos(t5) -sin(t5) 0 0;
%         0 0 -1 0;
%         sin(t5) cos(t5) 0 0;
%         0 0 0 1];
% T5_6 = [cos(t6) -sin(t6) 0 0;
%         0 0 1 0;
%        -sin(t6) -cos(t6) 0 0;
%         0 0 0 1];
% T0_6 = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6);
% T0_5 = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5);
% T0_4 = simplify(T0_1 * T1_2 * T2_3 * T3_4);
% T0_3 = simplify(T0_1 * T1_2 * T2_3);
% T0_2 = simplify(T0_1 * T1_2);
% % Rotational matrix
% R0_1 = T0_1(1:3, 1:3);
% R1_2 = T0_2(1:3, 1:3);
% R0_2 = T0_2(1:3, 1:3);
% R2_3 = T2_3(1:3, 1:3);
% R0_3 = T0_3(1:3, 1:3);
% R3_4 = T3_4(1:3, 1:3);
% R0_4 = T0_4(1:3, 1:3);
% R4_5 = T4_5(1:3, 1:3);
% R0_5 = T0_5(1:3, 1:3);
% R5_6 = T5_6(1:3, 1:3);
% R0_6 = T0_6(1:3, 1:3);
% 
% R1_0 = transpose(R0_1);
% R2_1 = transpose(R1_2);
% R3_2 = transpose(R2_3);
% R4_3 = transpose(R3_4);
% R5_4 = transpose(R4_5);
% R6_5 = transpose(R5_6);
% 
% % translational matrix [4 col of the transformation matrix]
% P0_1 = T0_1(1:3, 4);
% P1_2 = T0_2(1:3, 4);
% P0_2 = T0_2(1:3, 4);
% P2_3 = T0_3(1:3, 4);
% P0_3 = T0_3(1:3, 4);
% P3_4 = T0_4(1:3, 4);
% P0_4 = T0_4(1:3, 4);
% P4_5 = T0_5(1:3, 4);
% P0_5 = T0_5(1:3, 4);
% P5_6 = T0_6(1:3, 4);
% P0_6 = T0_6(1:3, 4);
%% Direct Differentiation
% Follow the similar precedure with previous example [3 col of the transformation matrix]
Z01 = T01(1:3, 3); % z-axis of frame 1 (Joint 1 axis)
Z02 = T02(1:3, 3); % z-axis of frame 2 (Joint 1 axis)
Z03 = T03(1:3, 3); % z-axis of frame 3 (Joint 1 axis)
Z04 = T04(1:3, 3); % z-axis of frame 4 (Joint 1 axis)
% Z05 = T05(1:3, 3); % z-axis of frame 5 (Joint 1 axis)
% Z06 = T06(1:3, 3); % z-axis of frame 6 (Joint 1 axis)
%%
Pe = T04(1:3,4);
J1 = [cross(Z01,(Pe-P01)); Z01]; 
J2 = [cross(Z02,(Pe-P02)); Z02]; 
J3 = [cross(Z03,(Pe-P03)); Z03]; 
J4 = [cross(Z04,(Pe-P04)); Z04]; 
% J5 = [cross(Z05,(Pe-P05)); Z05]; 
% J6 = [cross(Z06,(Pe-P06)); Z06]; 
J_DD = simplify([J1, J2, J3, J4]);
% det_J = (det(J_DD));

%%
function [T, Tfinal] = FK(alpha, a, d, theta) % Calculated FK (263A Final Exam code)
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