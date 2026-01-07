function getMLEHeatmapCPU(params, options)
    % 扫描范围
    theta1_range = linspace(-180, 180, 50); % 50x50 网格
    % theta2_range 将在 parfor 内部生成以避免广播开销
    
    n1 = length(theta1_range);
    n2 = 50; % theta2_range 长度
    
    mle_matrix = zeros(n2, n1); % 注意行列对应: 行是 y (theta2), 列是 x (theta1)
    
    % 仿真时间
    tspan_mle = [0, 60];
    delta = 1e-3;
    
    fprintf('  Starting Grid Scan (%dx%d). This may take a while...\n', n1, n2);
    
    % 并行计算
    parfor i = 1:n1
        col_mle = zeros(n2, 1);
        t1_val = theta1_range(i);
        
        % 在 Worker 内部生成 theta2_range，避免广播变量
        local_theta2_range = linspace(-180, 180, n2);
        
        for j = 1:n2
            t2_val = local_theta2_range(j);
            
            % 构造两个初始条件
            z0_base = [deg2rad(t1_val); deg2rad(t2_val); 0; 0];
            z0_pert = z0_base;
            z0_pert(1) = z0_pert(1) + delta;
            
            % 运行两次仿真
            [~, z_base] = fourLAdamsMoultonMethod(@(t,z) doublePendulumODE(t,z,params), tspan_mle, z0_base, options);
            [~, z_pert] = fourLAdamsMoultonMethod(@(t,z) doublePendulumODE(t,z,params), tspan_mle, z0_pert, options);
            
            % 计算末态距离
            z_end_base = z_base(end, :);
            z_end_pert = z_pert(end, :);
            
            dist_final = norm(z_end_base - z_end_pert);
            dist_initial = delta;
            
            % 计算 FTLE: lambda = (1/T) * ln(d(T)/d(0))
            T = tspan_mle(2);
            lambda = (1/T) * log(dist_final / dist_initial);
            
            % 过滤掉可能的负值或异常值 (仅保留正值用于展示混沌)
            if lambda < 0, lambda = 0; end
            
            col_mle(j) = lambda;
        end
        mle_matrix(:, i) = col_mle;
        fprintf('.'); % 进度条
        if mod(i, 10) == 0, fprintf('\n'); end
    end
    fprintf('\n  Scan Completed.\n');
    
    % 绘图
    figure('Name', 'MLE Parameter Scan', 'Color', 'w');
    % 重新生成 theta2_range 用于绘图
    theta2_range = linspace(-180, 180, n2);
    imagesc(theta1_range, theta2_range, mle_matrix);
    set(gca, 'YDir', 'normal'); % 确保 Y 轴方向正确
    colorbar;
    colormap('jet');
    xlabel('\theta_{10} (deg)');
    ylabel('\theta_{20} (deg)');
    title('Finite Time Lyapunov Exponent (FTLE) Heatmap');
    axis equal;
    xlim([-180, 180]);
    ylim([-180, 180]);
    
    fprintf('  Task 3 Completed.\n');
end