clear, clc, close all 

[ned2, numJoints] = ned2_import;
ned2.DataFormat = "row";
ned2.homeConfiguration;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TUTTE LE MURA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
wallLength = 1;   % Lunghezza in X [m]
wallHeight = 0.3;   % Altezza in Z [m]
wallThickness = 0.05; % Spessore in Y [m]
wallCenter = [0.25, -0.2, 0]; % Posizione centro muro [x y z]
wall = collisionBox(wallLength, wallThickness, wallHeight);  
wall.Pose = trvec2tform(wallCenter);  % posizioneMuro(x, y, z)
% wall superiore
% wall 1
wallLength1 =1;   % Lunghezza in X [m]
wallHeight1 = 0.05;   % Altezza in Z [m]
wallThickness1 = 0.3; % Spessore in Y [m]
a = 0.5;
wallCenter1 = [0.25, -0.35, a]; % Posizione centro muro [x y z]
wall1 = collisionBox(wallLength1, wallThickness1, wallHeight1);  
wall1.Pose = trvec2tform(wallCenter1);  % posizioneMuro(x, y, z)

%wall 4 --> PAVIMENTO 

wallLength4 = 1;   % Lunghezza in X [m]  scaffale
wallHeight4 = 0.05;   % Altezza in Z [m]
wallThickness4 = 0.3; % Spessore in Y [m]
wallCenter4 = [0.25, -0.35, 0.125]; % Posizione centro muro [x y z]
wall4 = collisionBox(wallLength4, wallThickness4, wallHeight4);  
wall4.Pose = trvec2tform(wallCenter4);  % posizioneMuro(x, y, z)

%muri dietro il robot

% wall 5 --> parete dx

wallLength5 = 0.05;   % Lunghezza in X [m]
wallHeight5 = 0.4;   % Altezza in Z [m]
wallThickness5 = 0.3; % Spessore in Y [m]
wallCenter5 = [-0.25, -0.35, 0.3]; % Posizione centro muro [x y z]
wall5 = collisionBox(wallLength5, wallThickness5, wallHeight5);  
wall5.Pose = trvec2tform(wallCenter5);  % posizioneMuro(x, y, z)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ned2.Gravity = [0 0 -9.81];
ee = "gripper_link" ;
[wksp,cfgs] = generateRobotWorkspace(ned2,{wall,wall1,wall4,wall5},IgnoreSelfCollision="on");
mIndexYoshikawa = manipulabilityIndex(ned2,cfgs,IndexType="yoshikawa");
%% 

show(ned2);
hold on
show(wall);
hold on
show(wall1);
hold on
show(wall4);
hold on
show(wall5);
hold on

showWorkspaceAnalysis(wksp,mIndexYoshikawa);
hold off
title("Yoshikawa index");
axis auto