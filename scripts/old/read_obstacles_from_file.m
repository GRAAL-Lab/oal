%% Read obstacles from file (each obstacle is on one block)
function obstacles = read_obstacles_from_file(filename)
    fid = fopen(filename, 'r');
    if fid < 0
        error('Could not open file %s', filename);
    end
    obstacles = [];
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line) || strcmp(line, '---')
            continue;
        end
        if startsWith(line, 'Obstacle_')
            pattern = 'Obstacle_([^_]+)_Position_([-+0-9.]+)_([-+0-9.]+)_Heading_([-+0-9.eE]+)_Velocity_([-+0-9.]+)_([-+0-9.]+)';
            tokens = regexp(line, pattern, 'tokens');
            if ~isempty(tokens)
                t = tokens{1};
                obs.name = t{1};
                obs.position = [str2double(t{2}), str2double(t{3})];
                obs.velocity = [str2double(t{5}), str2double(t{6})];

                % Define default rectangular vertices (Modify this as needed)
                w = 5;  % Default width
                h = 5;  % Default height
                obs.vertices = [
                    obs.position(1) - w/2, obs.position(2) - h/2;
                    obs.position(1) + w/2, obs.position(2) - h/2;
                    obs.position(1) + w/2, obs.position(2) + h/2;
                    obs.position(1) - w/2, obs.position(2) + h/2
                ];

                obstacles = [obstacles; obs];
            end
        end
    end
    fclose(fid);
end