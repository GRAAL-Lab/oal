%% Read node traces from file (expects Trace_ header and _Node_ lines)
function nodes = read_nodes_from_file(filename)
    fid = fopen(filename, 'r');
    if fid < 0
        error('Could not open file %s', filename);
    end
    nodes = [];
    current_positions = [];
    current_headings  = [];
    current_times     = [];
    current_info      = {};
    current_trace_name = '';
    
    mainPattern = '_Node_\d+_Position_([-+0-9.]+)_([-+0-9.]+)_Heading_([-+0-9.eE]+)_Time_([-+0-9.]+)_Speed_([-+0-9.]+)';
    
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line)
            continue;
        end
        if startsWith(line, 'Trace_')
            current_trace_name = strrep(line, 'Trace_', '');
            continue;
        end
        if strcmp(line, '---')
            if ~isempty(current_positions)
                new_trace.positions = current_positions;
                new_trace.headings  = current_headings;
                new_trace.times     = current_times;
                new_trace.info      = current_info;
                new_trace.trace_name = current_trace_name;
                nodes = [nodes; new_trace];
                current_positions = [];
                current_headings  = [];
                current_times     = [];
                current_info      = {};
            end
            continue;
        end
        
        tokensMain = regexp(line, mainPattern, 'tokens');
        if ~isempty(tokensMain)
            t = tokensMain{1};
            x = str2double(t{1});
            y = str2double(t{2});
            heading = str2double(t{3});
            time_val = str2double(t{4});
            
            info_str = sprintf('X: %.1f\nY: %.1f\nHeading: %.2f rad\nTime: %.3f', x, y, heading, time_val);
            
            current_positions = [current_positions; x, y];
            current_headings  = [current_headings; heading];
            current_times     = [current_times; time_val];
            current_info{end+1} = info_str;
        end
    end
    fclose(fid);
end