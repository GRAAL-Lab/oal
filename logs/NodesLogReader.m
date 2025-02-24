clear; clc; close all;
plot_nodes_scroll;

function plot_nodes_scroll()
    % Specify the file path (adjust as needed)
    filename = '/home/graal/graal_ws/oal/logs/test.txt'; 
    % Read all trace data from the file
    data = read_nodes_from_file(filename);
    
    % Determine global axis limits so that all plots share the same view
    [xlim_vals, ylim_vals] = find_global_limits(data);
    
    % Create figure and show the first trace
    fig = figure;
    idx = 1;
    plot_trace(idx, data, xlim_vals, ylim_vals);
    
    % Use left/right arrow keys to navigate between traces
    set(fig, 'KeyPressFcn', @(src, event) navigate(event, data, xlim_vals, ylim_vals));
end

function navigate(event, data, xlim_vals, ylim_vals)
    persistent idx;
    if isempty(idx)
        idx = 1;
    end
    if strcmp(event.Key, 'rightarrow')
        idx = min(idx + 1, length(data));
    elseif strcmp(event.Key, 'leftarrow')
        idx = max(idx - 1, 1);
    end
    clf;
    plot_trace(idx, data, xlim_vals, ylim_vals);
end

function plot_trace(idx, data, xlim_vals, ylim_vals)
    trace = data(idx);
    positions = trace.positions;
    headings  = trace.headings;
    info      = trace.info;
    trace_name = trace.trace_name;
    
    % Plot positions connected by a blue line with circle markers
    plot(positions(:,1), positions(:,2), 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    
    arrow_scale = 5;
    for i = 1:size(positions,1)
        x = positions(i,1);
        y = positions(i,2);
        theta = headings(i);  % Heading in radians
        dx = arrow_scale * cos(theta);
        dy = arrow_scale * sin(theta);
        % Draw a red arrow indicating the heading
        quiver(x, y, dx, dy, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);
        % Overplot a red circle; clicking shows node details
        plot(x, y, 'ro', 'MarkerSize', 8, 'ButtonDownFcn', @(src,event) show_info(info{i}));
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

function show_info(text)
    msgbox(text, 'Node Info');
end

function [xlim_vals, ylim_vals] = find_global_limits(data)
    xs = [];
    ys = [];
    for i = 1:length(data)
        xs = [xs; data(i).positions(:,1)];
        ys = [ys; data(i).positions(:,2)];
    end
    padding = 10;
    xlim_vals = [min(xs)-padding, max(xs)+padding];
    ylim_vals = [min(ys)-padding, max(ys)+padding];
end

function data = read_nodes_from_file(filename)
    fid = fopen(filename, 'r');
    if fid < 0
        error('Could not open file %s', filename);
    end
    data = [];  %#ok<AGROW>
    
    % Temporary storage for one trace block
    current_positions = [];
    current_headings  = [];
    current_info      = {};
    current_trace_name = '';
    
    % Patterns for main fields and optional obstacle info
    mainPattern = '_Node_\d+_Position_([-+0-9.]+)_([-+0-9.]+)_Heading_([-+0-9.eE]+)_Time_([-+0-9.]+)_Speed_([-+0-9.]+)';
    obsPattern = '_Obstacle_([^_]+)_Vx_([^_]+)';
    
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line)
            continue;
        end
        
        % Check for trace header (e.g., "Trace_FinalPath" or "Trace_")
        if startsWith(line, 'Trace_')
            current_trace_name = strrep(line, 'Trace_', '');
            continue;
        end
        
        % Separator indicates end of a trace block
        if strcmp(line, '---')
            if ~isempty(current_positions)
                new_trace.positions = current_positions;
                new_trace.headings  = current_headings;
                new_trace.info      = current_info;
                new_trace.trace_name = current_trace_name;
                data = [data; new_trace];  %#ok<AGROW>
                current_positions = [];
                current_headings  = [];
                current_info      = {};
                current_trace_name = '';
            end
            continue;
        end
        
        % Process node line: extract main fields
        tokensMain = regexp(line, mainPattern, 'tokens');
        if ~isempty(tokensMain)
            t = tokensMain{1};
            x = str2double(t{1});
            y = str2double(t{2});
            heading = str2double(t{3});
            time_val = str2double(t{4});
            speed = str2double(t{5});
            
            % Look for optional obstacle information
            tokensObs = regexp(line, obsPattern, 'tokens');
            if ~isempty(tokensObs)
                obs = tokensObs{1}{1};
                vx = str2double(tokensObs{1}{2});
                extra = sprintf('Obstacle: %s\nVx: %.1f', obs, vx);
            else
                extra = 'No Obstacle';
            end
            
            info_str = sprintf('X: %.1f\nY: %.1f\nHeading: %.2f rad\nTime: %.3f\nSpeed: %.1f\n%s',...
                x, y, heading, time_val, speed, extra);
            
            current_positions = [current_positions; x, y];
            current_headings  = [current_headings; heading];
            current_info{end+1} = info_str;
        end
    end
    
    % Store any remaining trace data
    if ~isempty(current_positions)
        new_trace.positions = current_positions;
        new_trace.headings  = current_headings;
        new_trace.info      = current_info;
        new_trace.trace_name = current_trace_name;
        data = [data; new_trace];
    end
    fclose(fid);
end
