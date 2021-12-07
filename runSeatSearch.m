close all;
clear;
clc;
%% Load environment
startPos = [15, 50, 0];
endPos = [85, 50, 0];
costMap1 = load("Maps/Stadium/costmap.mat").costMap;
costMap2 = load("Maps/Stadium/costmap.mat").costMap;
costMap3 = load("Maps/Stadium/costmapLv2.mat").costMap;
costMap4 = load("Maps/Stadium/costmapLv3.mat").costMap;
costMap5 = load("Maps/Stadium/costmapLv4.mat").costMap;
costMap6 = load("Maps/Stadium/costmapLv5.mat").costMap;
%% Handle cost changes in cost map
eh = csvread("Maps/Stadium_mod/allCostChanges_v3.csv");
changes = sortrows(eh);
map = containers.Map('KeyType', 'double', 'ValueType', 'any');

for i=5:5:100
    temp = [];
    for j=1:length(changes)
        if (changes(j, 1) == i)
            temp = [temp changes(j, 2:end)];
        end
    end
    map(i) = temp;
end
for i = 5:5:100
    map(i) = reshape(map(i), 4, [])';
end
s = map(5);
%% Get action (path) from planner.cpp
% a = DStarLite(costMap1, costMap2, costMap3, costMap4, costMap5, costMap6, startPos, endPos)
path = csvread("path.csv")
%path from demo:
old_path = [51,2,0;

51,3,0;

51,4,0;

52,5,0;

53,6,0;

54,7,0;

55,8,0;

55,9,0;

56,10,0;

57,11,0;

58,12,0;

57,11,0;

56,10,0;

57,11,0;

56,10,0;

57,11,0;

58,12,0;

57,11,0;

58,12,0;

57,11,0;

58,12,0;

59,11,0;

60,10,0;

61,9,0;

62,8,0;

63,9,0;

64,8,0;

63,9,0;

64,8,0;

63,9,0;

64,8,0;

63,9,0;

62,10,0;

63,9,0;

62,10,0;

63,9,0;

62,10,0;

62,11,0;

62,10,0;

62,11,0;

62,10,0;

61,11,0;

62,12,0;

63,13,0;

64,14,0;

65,15,0;

66,16,0;

67,17,0;

68,18,0;

69,19,0;

70,20,0;

69,19,0;

68,18,0;

69,19,0;

68,18,0;

69,19,0;

68,18,0;

69,19,0;

68,18,0;

69,19,0;

68,18,0;

69,19,0;

68,18,0;

69,19,0;

68,18,0;

69,19,0;

68,20,0;

68,21,0;

69,22,0;

70,23,0;

71,24,0;

72,25,0;

73,26,0;

74,27,0;

75,28,0;

75,29,0;

76,30,0;

77,31,0;

78,32,0;

78,33,0;

79,34,0;

80,35,0;

81,36,0;

82,37,0;

83,38,0;

84,39,0;

85,40,0;

86,41,0;

87,42,0;

88,43,0;

89,44,0;

90,45,0;

91,46,0;

92,47,0;

93,48,0;

94,49,0;

95,50,0;

96,51,0;

51,10,2;

8,51,4;

51,98,6;

98,51,8;

51,4,10;

52,3,10;

53,2,10;

54,1,10;

55,1,10;

56,1,10;

57,1,10;

58,1,10;

59,1,10;

60,1,10;

61,1,10;

62,1,10;

63,1,10;

64,2,10;

65,2,10;

66,2,10;

67,2,10;

68,3,10;

69,3,10;

70,4,10;

71,4,10;

72,5,10;

73,5,10;

74,6,10;

75,6,10;

76,7,10;

77,7,10;

78,8,10;

79,9,10;

80,10,10;

81,11,10];


X_path = path(:, 1);
Y_path = path(:, 2);
Z_path = path(:, 3);
%% Plot result on level maps
% figure('units','normalized');
% maxVal = max(costMap1, [], 'all');
% cmap = colormap(jet(maxVal));
% hold on;
costMapBase = ones(size(costMap1));
t = 1;
costMap1_o = costMap1;
costMap2_o = costMap2;
costMap3_o = costMap3;
costMap4_o = costMap4;
costMap5_o = costMap5;
costMap6_o = costMap6;
sumCost = 0;
while t < length(X_path)
    if (isKey(map, t))
        s = map(t); % s = [z, x, y, C]
        for i = 1:length(s)
            if s(i, 1) == 0
                costMap1(s(i, 2)+1, s(i, 3)+1) = costMap1(s(i, 2)+1, s(i, 3)+1) + 5*s(i, 4);
            elseif s(i, 1) == 2
                costMap2(s(i, 2)+1, s(i, 3)+1) = costMap2(s(i, 2)+1, s(i, 3)+1) + 5*s(i, 4);
            elseif s(i, 1) == 4
                costMap3(s(i, 2)+1, s(i, 3)+1) = costMap3(s(i, 2)+1, s(i, 3)+1) + 5*s(i, 4);
            elseif s(i, 1) == 6
                costMap4(s(i, 2)+1, s(i, 3)+1) = costMap4(s(i, 2)+1, s(i, 3)+1) + 5*s(i, 4);
            elseif s(i, 1) == 8
                costMap5(s(i, 2)+1, s(i, 3)+1) = costMap5(s(i, 2)+1, s(i, 3)+1) + 5*s(i, 4);
            elseif s(i, 1) == 10
                costMap6(s(i, 2)+1, s(i, 3)+1) = costMap6(s(i, 2)+1, s(i, 3)+1) + 5*s(i, 4);
            end          
        end
    end
%     warp(0*costMapBase, costMap1, cmap)
%     warp(2*costMapBase, costMap2, cmap)
%     warp(4*costMapBase, costMap3, cmap)
%     warp(6*costMapBase, costMap4, cmap)
%     warp(8*costMapBase, costMap5, cmap)
%     warp(10*costMapBase, costMap6, cmap)
%     plot3(X_path(t), Y_path(t), Z_path(t), 'o', 'Markersize', 10, 'MarkerFaceColor', [0.5,0.5,0.5])

    f1 = figure(1);
    f1.Position(1:4) = [400 0 1000 1000];
    hold on
    axis equal
    xlabel("x", 'FontSize', 30)
    ylabel("y", 'FontSize', 30)
    if Z_path(t) == 0
        sumCost = sumCost + costMap1(X_path(t), Y_path(t));
        axis equal
        imagesc(costMap1)
        plot3(X_path(t), Y_path(t), Z_path(t), 'o', 'Markersize', 10, 'MarkerFaceColor', [0.5,0.5,0.5])
        title("Floor = 0", 'FontSize', 30)
    elseif Z_path(t) == 2
        sumCost = sumCost + costMap2(X_path(t), Y_path(t));
        imagesc(costMap2)
        plot3(X_path(t), Y_path(t), Z_path(t), 'o', 'Markersize', 10, 'MarkerFaceColor', [0.5,0.5,0.5])
        title("Floor = 1", 'FontSize', 30)
    elseif Z_path(t) == 4
        sumCost = sumCost + costMap3(X_path(t), Y_path(t));
        imagesc(costMap3)
        plot3(X_path(t), Y_path(t), Z_path(t), 'o', 'Markersize', 10, 'MarkerFaceColor', [0.5,0.5,0.5])
        title("Floor = 2", 'FontSize', 30)
    elseif Z_path(t) == 6
        sumCost = sumCost + costMap4(X_path(t), Y_path(t));
        imagesc(costMap4)
        plot3(X_path(t), Y_path(t), Z_path(t), 'o', 'Markersize', 10, 'MarkerFaceColor', [0.5,0.5,0.5])
        title("Floor = 3", 'FontSize', 30)
    elseif Z_path(t) == 8
        sumCost = sumCost + costMap5(X_path(t), Y_path(t));
        imagesc(costMap5)
        plot3(X_path(t), Y_path(t), Z_path(t), 'o', 'Markersize', 10, 'MarkerFaceColor', [0.5,0.5,0.5])
        title("Floor = 4", 'FontSize', 30)
    elseif Z_path(t) == 10
        sumCost = sumCost + costMap6(X_path(t), Y_path(t));
        imagesc(costMap6)
        plot3(X_path(t), Y_path(t), Z_path(t), 'o', 'Markersize', 10, 'MarkerFaceColor', [0.5,0.5,0.5])
        title("Floor = 5", 'FontSize', 30)
    end    
    drawnow;
    pause(0.1)
    t = t+1;
    if (isKey(map, t))
        costMap1 = costMap1_o;
        costMap2 = costMap2_o;
        costMap3 = costMap3_o;
        costMap4 = costMap4_o;
        costMap5 = costMap5_o;
        costMap6 = costMap6_o;
    end
end
%% Display Cost
fprintf("Cost of traversal: %d\n", sumCost);
%% Stadium plot
% Initial Params
M = 10; % radius span splice amount
N = 100; % circumferential splice amount
R1 = 30; % inner radius 
R2 = 50;  % outer radius
nR = linspace(R1,R2,M); % splice radius
nT = linspace(0,2*pi,N); % splice theta

R1b = 34; % inner radius 
nRb = linspace(R1b,R2,M); % splice radius

R1c = 38; % inner radius 
nRc = linspace(R1c,R2,M); % splice radius

R1d = 42; % inner radius 
nRd = linspace(R1d,R2,M); % splice radius

R1e = 46; % inner radius 
nRe = linspace(R1e,R2,M); % splice radius

minLevel = 2;
maxLevel = 12;
[R, T] = meshgrid(nR,nT); % mesh polar coordinates
[Rb, Tb] = meshgrid(nRb,nT); % mesh polar coordinates
[Rc, Tc] = meshgrid(nRc,nT); % mesh polar coordinates
[Rd, Td] = meshgrid(nRd,nT); % mesh polar coordinates
[Re, Te] = meshgrid(nRe,nT); % mesh polar coordinates
offset = R2;
padding = 0;

% Convert grid to cartesian coordintes
X = R.*cos(T) + offset; 
Y = R.*sin(T) + offset;
Xb = Rb.*cos(Tb) + offset; 
Yb = Rb.*sin(Tb) + offset;
Xc = Rc.*cos(Tc) + offset; 
Yc = Rc.*sin(Tc) + offset;
Xd = Rd.*cos(Td) + offset; 
Yd = Rd.*sin(Td) + offset;
Xe = Re.*cos(Te) + offset; 
Ye = Re.*sin(Te) + offset;
[m,n] = size(X);
Z = zeros(m, n);
base = ones(m, n);

% Sloped height
for i=1:m
    Z(i,:) = linspace(minLevel, maxLevel, M);
end

% Stadium plot
% figure('units','normalized');
% box on
f2 = figure(2);
f2.Position(1:4) = [400 0 1000 1000];
hold on
% Seats
surf(X, Y, Z);
% Ground
surf(X, Y, 0*base);
surf(X, Y, 2*base);
surf(Xb, Yb, 4*base);
surf(Xc, Yc, 6*base);
surf(Xd, Yd, 8*base);
surf(Xe, Ye, 10*base);
% Outer wall
% [X_, Y_, Z_] = cylinder(R2, N);
% surf(X_ + offset, Y_+ offset, maxLevel*Z_)
% Inner wall
[X_, Y_, Z_] = cylinder(R1, N);
surf(X_ + offset, Y_+ offset, minLevel*Z_)
% Plot path
plot3(X_path, Y_path, Z_path, 'r', 'LineWidth', 5)
xlabel("x")
ylabel("y")
zlabel("z")