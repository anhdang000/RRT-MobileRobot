clear
close all

[start, goal, mapSize, mapMask, obstacles] = initMap();
rrt = RRTGraph(start, goal, mapMask, mapSize);

biasIter = 5;
iter = 0;
while ~rrt.goalFlag
    if mod(iter, biasIter) == 0
        rrt = rrt.bias(goal);
        if size(rrt.treeCoors, 1) ~= 1
            branch = [rrt.treeCoors(end, :); rrt.treeCoors(rrt.parent(end), :)];
            plot(branch(:, 1), branch(:, 2), 'c-.');
        end
    else
        rrt = rrt.expand();
        if size(rrt.treeCoors, 1) ~= 1
            branch = [rrt.treeCoors(end, :); rrt.treeCoors(rrt.parent(end), :)];
            plot(branch(:, 1), branch(:, 2), 'c-.');
        end
    end
    
    rrt = rrt.getPath2Goal();
    
    iter = iter + 1;
    drawnow
end

pathCoors = rrt.getPathCoors();
plot(pathCoors(:, 1), pathCoors(:, 2), 'k-.');

optimalPathCoors = rrt.optimizePath(pathCoors);
plot(optimalPathCoors(:, 1), optimalPathCoors(:, 2), ...
    'LineWidth', 2, 'Color', [0 0.4470 0.7410]);

save('variables/mapComponents.mat', 'start', 'goal', 'mapSize', 'mapMask', 'obstacles');
