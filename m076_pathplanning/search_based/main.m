% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
% addpath('A_star')

% Environment map in 2D space 
% 生成地图
xStart = 5;
yStart = 5;
MAX_X = 30;
MAX_Y = 15;
xTarget = MAX_X;
yTarget = MAX_Y;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y); % 返回障碍物出现的地方
  

% map = [[4, 6]; [1, 1; 15, 10];  [5*ones(6, 1), (3:8)']; [15, 6] ];
% map = [[4, 5]; [1, 1; 15, 10];  [5*ones(6, 1), (3:8)']; [15, 5] ];


% Waypoint Generator Using the Djistra
% path_dijkstra = ws_dijkstra(map);

% Waypoint Generator Using the Astar
% path_astar = ws_astar(map);

% Waypoint Generator Using the JPS
path_astar = ws_JPS(map);

