clear; clc; close all;
plot_nodes_and_obstacles;

%% Main function: read nodes & obstacles and plot both
function plot_nodes_and_obstacles()
    % Adjust file paths as needed
    nodesFile = '/home/graal/graal_ws/oal/logs/nodes.txt';
    obsFile   = '/home/graal/graal_ws/oal/logs/obstacles.txt';
    
    nodes = read_nodes_from_file(nodesFile);
    obstacles = read_obstacles_from_file(obsFile);
    
    [xlim_vals, ylim_vals] = find_global_limits(nodes);
    
    fig = figure;
    idx = 1;
    plot_trace(idx, nodes, obstacles, xlim_vals, ylim_vals);
    
    % Enable interactive navigation (left/right arrow keys)
    set(fig, 'KeyPressFcn', @(src, event) navigate(event, nodes, obstacles, xlim_vals, ylim_vals));
end

%% Navigation callback to scroll through node traces
function navigate(event, nodes, obstacles, xlim_vals, ylim_vals)
    persistent idx;
    if isempty(idx)
        idx = 1;
    end
    if strcmp(event.Key, 'rightarrow')
        idx = min(idx + 1, length(nodes));
    elseif strcmp(event.Key, 'leftarrow')
        idx = max(idx - 1, 1);
    end
    clf;  % Clear figure before redrawing
    plot_trace(idx, nodes, obstacles, xlim_vals, ylim_vals);
end

%% Plot a single node trace (with heading arrows and obstacles at the right positions)
function plot_trace(idx, nodes, obstacles, xlim_vals, ylim_vals)
    clf;  % Clear figure every time we plot

    % trace = nodes(idx);
    % positions = trace.positions;
    % headings  = trace.headings;
    % times     = trace.times;  % Extract time values
    % info      = trace.info;
    % trace_name = trace.trace_name;
    % 
    % current_time = times(end);  % Get the last time for this trace
    % 
    % % Plot node trace: blue line with circle markers
    % plot(positions(:,1), positions(:,2), 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
    % hold on;
    % 
    % % Draw heading arrows
    % arrow_scale = 5;
    % for i = 1:size(positions,1)
    %     x = positions(i,1);
    %     y = positions(i,2);
    %     theta = headings(i);
    %     dx = arrow_scale * cos(theta);
    %     dy = arrow_scale * sin(theta);
    %     quiver(x, y, dx, dy, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);
    %     plot(x, y, 'ro', 'MarkerSize', 8, 'ButtonDownFcn', @(src,event) show_info(info{i}));
    % end

    for i = 1:size(obstacles,1)
        plot_obstacle(obstacles(i),0);
    end

    for i = 1:size(positions,1)
       
    end

    axis equal;
    xlim(xlim_vals);
    ylim(ylim_vals);
    xlabel('X Position');
    ylabel('Y Position');
    grid on;

    if isempty(trace_name)
        title(sprintf('Trace %d', idx));
    else
        title(trace_name, 'Interpreter', 'none');
    end
    hold off;
end

%% Plot obstacles as filled polygons (rectangles defined by the vertices)
function plot_obstacle(obs, current_time)
    % Ensure obstacle has valid vertices
    if isempty(obs.vertices) || size(obs.vertices,2) < 2
        warning('Obstacle "%s" has invalid or missing vertices.', obs.name);
        return;
    end

    % Compute new position using velocity
    new_x = obs.position(1) + obs.velocity(1) * current_time;
    new_y = obs.position(2) + obs.velocity(2) * current_time;

    % Compute translated vertices safely
    verts = obs.vertices;  % Copy original vertices
    verts(:,1) = verts(:,1) + (new_x - obs.position(1));  % Update X coordinates
    verts(:,2) = verts(:,2) + (new_y - obs.position(2));  % Update Y coordinates

    % Plot obstacle as a yellow polygon with black edges
    patch(verts(:,1), verts(:,2), 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1.5);

    % Label the obstacle
    text(new_x, new_y, obs.name, 'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold');
    
end


%% Popup to display node information when a node is clicked
function show_info(text)
    msgbox(text, 'Node Info');
end

%% Determine global axis limits from all node traces
function [xlim_vals, ylim_vals] = find_global_limits(nodes)
    xs = [];
    ys = [];
    for i = 1:length(nodes)
        xs = [xs; nodes(i).positions(:,1)];
        ys = [ys; nodes(i).positions(:,2)];
    end
    padding = 10;
    xlim_vals = [min(xs)-padding, max(xs)+padding];
    ylim_vals = [min(ys)-padding, max(ys)+padding];
end





