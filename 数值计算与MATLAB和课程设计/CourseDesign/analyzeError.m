function analyzeError(odeFunc, tspan, z0, params, solversList)
    % analyzeError 分析数值方法的误差特性
    
    fprintf('正在计算高精度参考解 (ode45, RelTol=1e-13)...\n');
    ref_options = odeset('RelTol', 1e-13, 'AbsTol', 1e-14);
    sol_ref = ode45(@(t,z) odeFunc(t,z,params), tspan, z0, ref_options);
    
    % 准备绘图
    fig_time = figure('Name', 'Error vs Time', 'Color', 'w');
    ax_time = axes(fig_time); hold on; grid on;
    xlabel('Time (s)'); ylabel('Global Error (Norm)');
    set(gca, 'YScale', 'log');
    title('误差随时间累积 (Fixed Step)');
    
    fig_conv = figure('Name', 'Convergence Analysis', 'Color', 'w');
    ax_conv = axes(fig_conv); hold on; grid on;
    xlabel('Step Size h (s)'); ylabel('Max Global Error');
    set(gca, 'XScale', 'log', 'YScale', 'log');
    title('误差随步长变化 (Convergence)');
    
    % 颜色映射
    colors = lines(size(solversList, 1));
    
    % 分析参数
    h_fixed_for_time_plot = 0.001; % 用于"误差-时间"图的固定步长
    % 0.1 和 0.05 对于显式方法通常太大了，会导致发散和矩阵奇异
    % 移除过大的步长以避免报错和浪费时间
    h_values = [0.01, 0.005, 0.002, 0.001, 0.0005]; 
    
    for i = 1:size(solversList, 1)
        solver = solversList{i, 1};
        name = solversList{i, 2};
        color = colors(i, :);
        
        fprintf('正在分析求解器: %s ...\n', name);
        
        %% 1. 误差随时间变化 (Error vs Time)
        try
            opts = odeset('MaxStep', h_fixed_for_time_plot);
            
            % 临时关闭警告
            warnState = warning('off', 'MATLAB:singularMatrix');
            warning('off', 'MATLAB:nearlySingularMatrix');
            
            [t_sol, z_sol] = solver(@(t,z) odeFunc(t,z,params), tspan, z0, opts);
            
            % 恢复警告
            warning(warnState);
            
            if any(isnan(z_sol(:))) || any(isinf(z_sol(:)))
                error('Solution diverged (NaN/Inf)');
            end
            
            z_true = deval(sol_ref, t_sol).';
            err = sqrt(sum((z_sol - z_true).^2, 2));
            plot(ax_time, t_sol, err, 'Color', color, 'LineWidth', 1.5, 'DisplayName', name);
        catch ME
            warning(warnState); % 确保恢复
            fprintf('  [跳过] %s 在 h=%.4f 时发散或出错: %s\n', name, h_fixed_for_time_plot, ME.message);
        end
        
        %% 2. 误差随步长变化 (Convergence)
        max_errors = [];
        valid_hs = [];
        
        for h = h_values
            try
                opts = odeset('MaxStep', h);
                
                warnState = warning('off', 'MATLAB:singularMatrix');
                warning('off', 'MATLAB:nearlySingularMatrix');
                
                [t_sol, z_sol] = solver(@(t,z) odeFunc(t,z,params), tspan, z0, opts);
                
                warning(warnState);
                
                if any(isnan(z_sol(:))) || any(isinf(z_sol(:)))
                    error('Diverged');
                end
                
                z_true = deval(sol_ref, t_sol).';
                err = sqrt(sum((z_sol - z_true).^2, 2));
                
                max_err = max(err);
                if max_err > 1e3 % 误差过大也视为发散
                    error('Large Error');
                end
                
                max_errors(end+1) = max_err;
                valid_hs(end+1) = h;
            catch
                warning(warnState); % 确保恢复
                % 忽略发散的步长
            end
        end
        
        if ~isempty(valid_hs)
            % 线性拟合 log(error) = p * log(h) + c
            p_coeffs = polyfit(log(valid_hs), log(max_errors), 1);
            slope = p_coeffs(1);
            intercept = p_coeffs(2);
            
            % 更新图例显示拟合阶数
            displayName = sprintf('%s (Order \\approx %.2f)', name, slope);
            plot(ax_conv, valid_hs, max_errors, 'o', 'Color', color, 'LineWidth', 1.5, 'DisplayName', displayName);
            
            % 绘制拟合直线 (虚线)
            fit_errors = exp(polyval(p_coeffs, log(valid_hs)));
            plot(ax_conv, valid_hs, fit_errors, '--', 'Color', color, 'LineWidth', 1.0, 'HandleVisibility', 'off');
            
            fprintf('  %s 估计收敛阶: %.2f\n', name, slope);
        end
    end
    
    legend(ax_time, 'Location', 'best');
    legend(ax_conv, 'Location', 'best');
end