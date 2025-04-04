%% Main Script Example

clear; clc; close all;

% Specify the file name (it should contain your obstacle data)
filename = '/home/graal/ros2_ws/log/avoidance_logs/obstacles/obstacles.txt';

% Parse the file to obtain an array of obstacle structures
obstacles = parseObstacleFile(filename);

% Define a time offset (t = 0 means original positions)
t = 5;  % example: update positions 5 seconds later

% Plot the obstacles at time t
figure;
plotObstacles(obstacles, t);

% Print pretty obstacle information for time t
printObstacleInfo(obstacles, t);

%% Function Definitions

function obstacles = parseObstacleFile(filename)
    % Open the file for reading
    fid = fopen(filename, 'r');
    if fid == -1
        error('Could not open file: %s', filename);
    end

    obstacles = [];
    
    % Read the file line by line
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        % Skip empty lines or lines that are just separator '---'
        if isempty(line) || strcmp(line, '---')
            continue;
        end
        % Check if the line starts with 'Obstacle'
        if startsWith(line, 'Obstacle')
            % Split the line using underscore as delimiter
            tokens = strsplit(line, '_');
            % Expected token order:
            % { 'Obstacle', id, 'Position', posX, posY, 'Heading', heading, 
            %   'Velocity', velX, velY, 'Vx0', vx0_x, vx0_y, 'Vx1', vx1_x, vx1_y,
            %   'Vx2', vx2_x, vx2_y, 'Vx3', vx3_x, vx3_y }
            obs.id = str2double(tokens{2});
            obs.pos = [str2double(tokens{4}), str2double(tokens{5})];
            obs.heading = str2double(tokens{7});
            obs.velocity = [str2double(tokens{9}), str2double(tokens{10})];
            % There are 4 vertices. Preallocate a 4-by-2 array.
            vertices = zeros(4,2);
            for k = 0:3
                % For each vertex, the tokens are: label, x, y.
                baseIdx = 11 + k*3;  % token index for label (e.g., 'Vx0')
                vx = str2double(tokens{baseIdx+1});
                vy = str2double(tokens{baseIdx+2});
                vertices(k+1,:) = [vx, vy];
            end
            obs.vertices = vertices;
            % Append the parsed obstacle to the array
            obstacles = [obstacles; obs];
        end
    end
    fclose(fid);
end

function plotObstacles(obstacles, t)
    % Open a figure and hold for multiple plots
    hold on;
    for i = 1:length(obstacles)
        obs = obstacles(i);
        
        % Update the position and vertices if t > 0
        % New positions = original + velocity * t
        newCenter = obs.pos + obs.velocity * t;
        newVertices = obs.vertices + obs.velocity * t;

        newVertices = [newVertices(2,:); newVertices(1,:); newVertices(3,:); newVertices(4,:)];
       
        % Plot the rectangle defined by the updated vertices.
        % Close the polygon by repeating the first vertex at the end.
        patch([newVertices(:,1); newVertices(1,1)], [newVertices(:,2); newVertices(1,2)], ...
              'b', 'FaceAlpha', 0.3, 'EdgeColor', 'b');
        
        % Add the obstacle id near the updated center position.
        text(newCenter(1), newCenter(2), sprintf('%d', obs.id), ...
             'Color', 'r','FontWeight', 'bold');
         
        % Draw an arrow indicating the obstacle’s velocity.
        % The arrow originates at the updated center.
        quiver(newCenter(1), newCenter(2), obs.velocity(1), obs.velocity(2), ...
               0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 1);
    end
    hold off;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title(sprintf('Obstacles at t = %.2f', t));
end

function printObstacleInfo(obstacles, t)
    % Print a header for clarity.
    fprintf('Obstacle Information at time t = %.2f seconds:\n', t);
    fprintf('--------------------------------------------------\n');
    for i = 1:length(obstacles)
        obs = obstacles(i);
        % Update the center position based on the velocity and time offset.
        newCenter = obs.pos + obs.velocity * t;
        % Format and print the information
        fprintf('Obstacle %d:\n', obs.id);
        fprintf('   Updated Position: (%.2f, %.2f)\n', newCenter(1), newCenter(2));
        fprintf('   Heading: %.2f radians\n', obs.heading);
        fprintf('   Velocity: (%.2f, %.2f)\n', obs.velocity(1), obs.velocity(2));
        fprintf('\n');
    end
end
