clear
close all

[start, goal, mapSize, mapMatrix] = initMap();
rrt = RRTGraph(start, goal, mapMatrix, mapSize);

biasIter = 4;
iter = 0;
while ~rrt.goalFlag
    if mod(iter, biasIter) == 0
        rrt = rrt.bias(goal);
        plot(rrt.treeCoors(end, 1), rrt.treeCoors(end, 2), 'c.');
    else
        rrt = rrt.expand();
        plot(rrt.treeCoors(end, 1), rrt.treeCoors(end, 2), 'c.');
    end
    
    rrt = rrt.getPath2Goal();
    
    iter = iter + 1;
    drawnow
end

pathCoors = rrt.getPathCoors();
plot(pathCoors(:, 1), pathCoors(:, 2), 'LineWidth', 2)

