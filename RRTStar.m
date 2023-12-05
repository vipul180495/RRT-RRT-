
clearvars
close all

% Initialization
% for center rectangle 
%{
x_max = 100;
y_max = 100;
obstacle = [30,30,40,40];
EPS = 2; % d, distance between nearest to new point
numNodes = 1000;        

q_start.coord = [0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [90 90];
q_goal.cost = 0;
%}

% for two obstacles
x_max = 100;
y_max = 100;
obstacle1 = [20,20,10,100];
obstacle2 = [70,0,10,80];
EPS = 10; % d, distance between nearest to new point
numNodes = 2000;        

q_start.coord = [5 90];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [95 10];
q_goal.cost = 0;

nodes(1) = q_start;   % creating tree

% Plotting obstacles and start/goal pts
figure(1)
axis([0 x_max 0 y_max])
rectangle('Position',obstacle1,'FaceColor',[0 .5 .5])
hold on
rectangle('Position',obstacle2,'FaceColor',[0 .5 .5])
hold on


for i = 1:1:numNodes
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];  % creating random points
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.0410])
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx); % found new point/nearest pt
    
    q_new.coord = steer(q_rand, q_near.coord, val, EPS); % new point
    if noCollision(q_rand, q_near.coord, [obstacle1; obstacle2])
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        %RRT* start
        % Within a radius of r, find all existing nodes
        % rewiring
        q_nearest = [];
        r = 10;   % radius for rewiring
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollision(nodes(j).coord, q_new.coord, [obstacle1; obstacle2]) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
                
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower cost paths        
        for k = 1:1:length(q_nearest)
            if noCollision(q_nearest(k).coord, q_new.coord, [obstacle1; obstacle2]) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                hold on
            end
        end        
        % RRT* end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        %q_new.parent = idx;
        % Append to nodes
        nodes = [nodes q_new];
    end
end

% Backtracking and path visulization
D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodes(start);
end
