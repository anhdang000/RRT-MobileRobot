function [start, goal, mapSize, mapMask, obstacles] = loadMapFromFile(filename)
load(filename);

close all
figure;
axis equal
grid on
hold on

xlim([0 4]);
ylim([0 4]);
xticks(0:1:4);
yticks(0:1:4);

% Plot start and goal
plot(start(1), start(2), 'o', 'MarkerFaceColor', [0 0.4470 0.7410], ...
                'MarkerSize', 10, 'Color', [0 0.4470 0.7410]);
plot(goal(1), goal(2), 's', 'MarkerFaceColor', [0.4660 0.6740 0.1880], ...
                'MarkerSize', 10, 'Color', [0.4660 0.6740 0.1880]);

% Obstacles
numObs = size(obstacles, 1);
for i=1:numObs
    rectangle('Position', [obstacles(i, 1) obstacles(i, 2) obstacles(i, 3) obstacles(i, 4)], ...
        'FaceColor',[1 0.9 0]);
end

end

