function map_matrix = init_map()
figure;
axis equal
grid on
hold on

xlim([0 4]);
ylim([0 4]);

map_size = [4, 4]; % [width, height]
start = [0.5 0.5];
goal = [3.5, 3.5];

%  Create map matrix
map_matrix = zeros(map_size(1)*100, map_size(2)*100);
map_matrix(start(1)*100-10:start(1)*100+10, start(2)*100-10:start(2)*100+10) = 1;
map_matrix(goal(1)*100-10:goal(1)*100+10, goal(2)*100-10:goal(2)*100+10) = 1;

% Plot start and goal
plot(start(1), start(2), 'o', 'MarkerFaceColor', [0 0.4470 0.7410], ...
                'MarkerSize', 10, 'Color', [0 0.4470 0.7410]);
plot(goal(1), goal(2), 's', 'MarkerFaceColor', [0.4660 0.6740 0.1880], ...
                'MarkerSize', 10, 'Color', [0.4660 0.6740 0.1880]);

% Obstacles
obs_dims = [1.1 0.12;
            0.3 0.2];
num_obs = 5;
obs_choices = [1, 1, 2, 2, 2];
for i=1:num_obs
   obs_dim = obs_dims(obs_choices(i), :);
   
   % Random position (upper_x, upper_y)
   while 1
       upper_x = randi([5, map_size(1)*100 - obs_dim(1)*100]);
       upper_y = randi([5, map_size(2)*100 - obs_dim(2)*100]);
       lower_x = floor(upper_x+obs_dim(1)*100);
       lower_y = floor(upper_y+obs_dim(2)*100);
       if max(map_matrix(upper_y:lower_y, upper_x:lower_x), [], 'all') == 0
           map_matrix(upper_y:lower_y, upper_x:lower_x) = 1;
           rectangle('Position',[upper_x/100 upper_y/100 obs_dim(1) obs_dim(2)], 'FaceColor',[1 0.9 0]);
           break
       end
   end
end
figure
imshow(map_matrix);
end

