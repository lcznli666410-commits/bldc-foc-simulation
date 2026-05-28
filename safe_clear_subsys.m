function safe_clear_subsys(subsys)
%% SAFE_CLEAR_SUBSYS 安全清除子系统默认内容
%  兼容不同版本的MATLAB/Simulink
%  某些版本的SubSystem默认包含In1/Out1端口和连线，某些版本则为空

    % 获取子系统内所有块
    try
        blocks = find_system(subsys, 'SearchDepth', 1, 'Type', 'block');
    catch
        return;  % 如果无法搜索，直接返回
    end
    
    % 获取子系统内所有连线并删除
    try
        lines = find_system(subsys, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
        for i = 1:length(lines)
            try
                delete_line(lines(i));
            catch
                % 忽略删除连线错误
            end
        end
    catch
        % 忽略
    end
    
    % 删除子系统内所有默认块 (排除子系统本身)
    for i = 1:length(blocks)
        blockPath = blocks{i};
        % 跳过子系统本身
        if strcmp(blockPath, subsys)
            continue;
        end
        try
            delete_block(blockPath);
        catch
            % 忽略删除块错误
        end
    end
end
