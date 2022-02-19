function [start, goal, mapSize, mapMatrix] = initMap()
close all
figure;
axis equal
grid on
hold on

xlim([0 4]);
ylim([0 4]);

mapSize = [4, 4]; % [width, height]
start = [0.5 0.5];
goal = [3.5, 3.5];

%  Create map matrix
mapMatrix = zeros(mapSize(1)*100, mapSize(2)*100);
mapMatrix(start(1)*100-10:start(1)*100+10, start(2)*100-10:start(2)*100+10) = 1;
mapMatrix(goal(1)*100-10:goal(1)*100+10, goal(2)*100-10:goal(2)*100+10) = 1;

% Plot start and goal
plot(start(1), start(2), 'o', 'MarkerFaceColor', [0 0.4470 0.7410], ...
                'MarkerSize', 10, 'Color', [0 0.4470 0.7410]);
plot(goal(1), goal(2), 's', 'MarkerFaceColor', [0.4660 0.6740 0.1880], ...
                'MarkerSize', 10, 'Color', [0.4660 0.6740 0.1880]);

% Obstacles
obsDims = [1.1 0.12; 0.3 0.2];
numObs = 5;
obsChoices = [1, 1, 2, 2, 2];
for i=1:numObs
   obsDim = obsDims(obsChoices(i), :);
   
   % Random position (upper_x, upper_y)
   while 1
       upper_x = randi([5, mapSize(1)*100 - obsDim(1)*100]);
       upper_y = randi([5, mapSize(2)*100 - obsDim(2)*100]);
       lower_x = floor(upper_x+obsDim(1)*100);
       lower_y = floor(upper_y+obsDim(2)*100);
       if max(mapMatrix(upper_y:lower_y, upper_x:lower_x), [], 'all') == 0
           mapMatrix(upper_y:lower_y, upper_x:lower_x) = 1;
           rectangle('Position',[upper_x/100 upper_y/100 obsDim(1) obsDim(2)], 'FaceColor',[1 0.9 0]);
           break
       end
   end
end

mapMatrix(start(1)*100-10:start(1)*100+10, start(2)*100-10:start(2)*100+10) = 0;
mapMatrix(goal(1)*100-10:goal(1)*100+10, goal(2)*100-10:goal(2)*100+10) = 0;
end

