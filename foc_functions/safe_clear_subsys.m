function safe_clear_subsys(subsys)
%SAFE_CLEAR_SUBSYS Remove default blocks and lines from a new subsystem.
% Different Simulink releases create slightly different default contents.

    try
        lines = find_system(subsys, ...
            'FindAll', 'on', ...
            'SearchDepth', 1, ...
            'Type', 'line');
        for idx = 1:numel(lines)
            try
                delete_line(lines(idx));
            catch
            end
        end
    catch
    end

    try
        blocks = find_system(subsys, 'SearchDepth', 1, 'Type', 'block');
    catch
        return;
    end

    for idx = 1:numel(blocks)
        blockPath = blocks{idx};
        if strcmp(blockPath, subsys)
            continue;
        end

        try
            delete_block(blockPath);
        catch
        end
    end
end
