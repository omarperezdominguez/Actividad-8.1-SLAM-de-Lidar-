%% EXAMPLE: Differential Drive Path Following
% In this example, a differential drive robot navigates a set of waypoints 
% using the Pure Pursuit algorithm while avoiding obstacles using the
% Vector Field Histogram (VFH) algorithm.
% 
% Copyright 2019 The MathWorks, Inc.

%% Simulation setup
clear all
close all
clc

% --- CARGA Y CONVERSIÓN DEL MAPA ---
load exampleMaps.mat
% Convertimos la matriz lógica 'complexMap' en un objeto de mapa real
% El '1' indica la resolución (1 celda = 1 metro)
map = binaryOccupancyMap(complexMap, 1); 

% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);

% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:61;        % Time array

% Initial conditions
initPose = [7; 2; pi];          % Initial pose (x y theta)
pose = zeros(3, numel(tVec));  % Pose matrix
pose(:,1) = initPose;

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0, 0];
lidar.scanAngles = linspace(-pi, pi, 200); 
lidar.maxRange = 1;
lidar.mapName = 'map';         % <-- Apunta al objeto 'map' recién creado

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';           % <-- Apunta al objeto 'map' recién creado
attachLidarSensor(viz, lidar);

%% Path planning and following
% Create waypoints (El patrón en zigzag)
waypoints = [initPose(1:2)'; 
                7 2;
                7 9;
                17 9;
                16 19;
                

                3 18;
                16 15
                17 9;
                26 9;
                

                22 17;
                27 9;
                26 5;

                24 1];

% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 2; 
controller.MaxAngularVelocity = 30;

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 3]; 
vfh.NumAngularSectors = 36; 
vfh.HistogramThresholds = [5 10]; 
vfh.RobotRadius = L;
vfh.SafetyDistance = L;
vfh.MinTurningRadius = 0.1;

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef, wRef, lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2), lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges, lidar.scanAngles, targetDir);    
    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5 * steerDir;
    end
    
    % Control the robot
    velB = [vRef; 0; wRef];           % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB, curPose); % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx), waypoints, ranges);
    waitfor(r);
end