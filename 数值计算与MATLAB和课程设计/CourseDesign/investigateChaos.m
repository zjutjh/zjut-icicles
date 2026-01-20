function investigateChaos()
    clc; close all;
    
    % 1. 基础参数设置
    % 载入默认参数 (仅为了获取 g, l, m 等物理常数，角度后面会覆盖)
    run('initalCondition.m'); 
    params = struct('l1', l1, 'l2', l2, 'm1', m1, 'm2', m2, 'g', g, 'theta10', theta10, 'theta20', theta20);
    
    % 求解器设置 (高精度)
    % 混沌对误差极其敏感，需要较小的步长和较高的容差
    options = odeset('RelTol', 1e-10, 'AbsTol', 1e-12, 'MaxStep', 0.001); 
    
    %% 任务 1: 相空间分离 (Phase Space Separation) - 蝴蝶效应可视化
    fprintf('Running Task 1: Phase Space Separation...\n');
    tspan = [0, 60]; % 运行时间
    getPhaseSpaceSeparation(tspan, params, options);
    

    %% 任务 2: 庞加莱截面 (Poincaré Section)
    fprintf('Running Task 2: Poincaré Section...\n');
    tspan_poincare = [0, 600];
    getPoincareSection(tspan_poincare, params, options);

    %% 任务 3: 最大李雅普诺夫指数 (MLE) 参数扫描
    fprintf('Running Task 3: MLE Parameter Scan (Heatmap)...\n');
    getMLEHeatmap(params, options);

end

%% 相空间分离图
function getPhaseSpaceSeparation(tspan, params, options)
    % 准备两个初始条件：Base 和 Perturbed
    z0_base = [deg2rad(params.theta10); deg2rad(params.theta20); 0; 0];
    
    delta = 1e-3; % 微扰大小
    z0_pert = z0_base;
    z0_pert(1) = z0_pert(1) + delta; % 仅在 theta1 上施加微扰
    odeFuncWrapped = @(t, z) doublePendulumODE(t, z, params);
    
    % 运行仿真
    fprintf('Simulating Base & Perturbed Trajectories...\n');
    [t_base, z_base] = fourLAdamsMoultonMethod(odeFuncWrapped, tspan, z0_base, options);
    [t_pert, z_pert] = fourLAdamsMoultonMethod(odeFuncWrapped, tspan, z0_pert, options);
    
    % 检查长度，取最小长度进行截断对比
    len = min(length(t_base), length(t_pert));
    t = t_base(1:len);
    z_base = z_base(1:len, :);
    z_pert = z_pert(1:len, :);
    
    % 计算相空间距离 (欧几里得距离)
    dp = z_base - z_pert;
    dist = sqrt(sum(dp.^2, 2));
    
    % 绘图
    figure('Name', 'Phase Space Separation', 'Color', 'w', 'Position', [100, 100, 1000, 600]);
    
    % 子图 1: 距离随时间的演化 (对数坐标)
    subplot(2, 1, 1);
    semilogy(t, dist, 'Color','#0076a8', 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Separation Distance ||\delta z||');
    title(sprintf('Phase Space Separation (Log Scale)\nInitial: \\theta_1=%d^\\circ, \\theta_2=%d^\\circ, \\delta=10^{-3}', params.theta10, params.theta20));
    
    % 添加参考线：如果斜率为正，说明指数分离
    
    % 子图 2: 两个轨迹的 theta1 对比，直观展示分离时刻
    subplot(2, 1, 2);
    plot(t, rad2deg(z_base(:,1)), 'Color','#0070bb', 'LineWidth', 1.0, 'DisplayName', 'Base');
    hold on;
    plot(t, rad2deg(z_pert(:,1)), 'Color','#cf2c1d', 'LineWidth', 1.0, 'DisplayName', 'Perturbed');
    grid on;
    xlabel('Time (s)');
    ylabel('\theta_1 (deg)');
    legend('Location', 'best');
    title('Trajectory Divergence in Time Domain');
    fprintf('  Task 1 Completed.\n');
end

%% 庞加莱截面图
function getPoincareSection(tspan_poincare, params, options)
    % 使用与上面相同的初始条件
    z0 = [deg2rad(params.theta10); deg2rad(params.theta20); 0; 0];
    
    fprintf('  Simulating for Poincaré Section (Duration: %ds)...\n', tspan_poincare(2));
    
    % 封装 ODE 函数
    odeFuncWrapped = @(t, z) doublePendulumODE(t, z, params);
    
    [t, z] = fourLAdamsMoultonMethod(odeFuncWrapped, tspan_poincare, z0, options);
    
    % 寻找截面穿越点: theta2 = 0 (mod 2pi) 且 theta2_dot > 0；即，监测 sin(theta2) 从负变正的时刻 (对应 theta2 = 2k*pi)
    
    % 预分配内存以优化性能
    max_points = length(t);
    poincare_points = zeros(max_points, 2);
    count = 0;
    
    theta2 = z(:, 2);
    % theta2_dot = z(:, 4);
    
    monitor_var = sin(theta2); 
    
    for i = 1:length(t)-1
        % 检测 monitor_var 从负变正 (Zero Crossing with Positive Slope)
        if monitor_var(i) < 0 && monitor_var(i+1) >= 0
            % 线性插值寻找精确时刻
            % dt = t(i+1) - t(i);
            dy = monitor_var(i+1) - monitor_var(i);
            frac = (0 - monitor_var(i)) / dy;
            
            % 插值状态
            z_cross = z(i, :) + frac * (z(i+1, :) - z(i, :));
            
            % 检查 theta2_dot > 0 (确保是同向穿越)
            if z_cross(4) > 0
                % 保存 (theta1, theta1_dot)
                % 将 theta1 归一化到 [-pi, pi]
                th1 = mod(z_cross(1) + pi, 2*pi) - pi;
                th1_dot = z_cross(3);
                
                count = count + 1;
                poincare_points(count, :) = [th1, th1_dot];
            end
        end
    end
    
    % 截断未使用的部分
    poincare_points = poincare_points(1:count, :);
    
    % 绘图
    figure('Name', 'Poincaré Section', 'Color', 'w');
    plot(poincare_points(:,1), poincare_points(:,2), '.', 'Color', '#0076a8', 'MarkerSize', 8);
    xlabel('\theta_1 (rad)');
    ylabel('$\dot{\theta}_1$ (rad/s)', 'Interpreter', 'latex');
    title(sprintf('Poincaré Section:\nInitial: \\theta_2=0^\\circ, \\theta_2>0^\\circ, Time=0-%d s', tspan_poincare(2)));
    grid on;
    xlim([-pi, pi]);
    
    fprintf('  Task 2 Completed. Found %d points.\n', size(poincare_points, 1));
end

%% MLE 热力图 (GPU 加速版)
function getMLEHeatmap(params, options)
    % 扫描范围 (可以适当增加分辨率，因为 GPU 很快)
    grid_res = 100; % 200x200 = 40000 个点
    theta1_range = linspace(-180, 180, grid_res);
    theta2_range = linspace(-180, 180, grid_res);
    
    [T1, T2] = meshgrid(theta1_range, theta2_range);
    
    % 仿真设置
    t_total = 60;
    dt = 0.005; % 固定步长 RK4 (适当减小步长以保证精度)
    steps = ceil(t_total / dt);
    delta = 1e-3;
    
    useGPU = (gpuDeviceCount > 0);
    
    if useGPU
        fprintf('  GPU detected. Using Vectorized RK4 on GPU for %d points...\n', numel(T1));
        
        % 1. 准备 GPU 数据
        % 状态向量: 4 x N (N = grid_res^2)
        % z = [th1; th2; w1; w2]
        N = numel(T1);
        
        % Base Trajectory
        z_base = gpuArray(zeros(4, N));
        z_base(1, :) = deg2rad(T1(:)');
        z_base(2, :) = deg2rad(T2(:)');
        
        % Perturbed Trajectory
        z_pert = z_base;
        z_pert(1, :) = z_pert(1, :) + delta;
        
        % 参数传到 GPU
        p.l1 = params.l1; p.l2 = params.l2;
        p.m1 = params.m1; p.m2 = params.m2; p.g = params.g;
        
        % 2. 时间步进 (Vectorized RK4)
        % 为了节省显存，我们不存储历史，只迭代更新当前状态
        
        % 预编译 ODE 函数句柄以减少开销
        ode = @(z) doublePendulumODE_Vectorized(z, p);
        
        % 显示进度条
        fprintf('  Progress: 0%%');
        for i = 1:steps
            z_base = rk4_step(z_base, dt, ode);
            z_pert = rk4_step(z_pert, dt, ode);
            
            if mod(i, floor(steps/10)) == 0
                fprintf('...%d%%', round(i/steps*100));
            end
        end
        fprintf('\n');
        
        % 3. 计算 MLE
        % 此时 z_base 和 z_pert 已经是 t_total 时刻的状态
        diff = z_base - z_pert;
        dist_final = sqrt(sum(diff.^2, 1)); % 欧几里得距离
        dist_initial = delta;
        
        lambda = (1/t_total) * log(dist_final / dist_initial);
        lambda(lambda < 0) = 0;
        
        % 取回 CPU
        mle_matrix = gather(reshape(lambda, grid_res, grid_res));
        
    else
        fprintf('  No GPU detected. Falling back to CPU parfor...\n');
        % 原有的 CPU 代码逻辑 (简化版，仅作 fallback)
        n1 = length(theta1_range);
        n2 = length(theta2_range);
        mle_matrix = zeros(n2, n1);
        parfor i = 1:n1
            col_mle = zeros(n2, 1);
            t1_val = theta1_range(i);
            local_theta2_range = linspace(-180, 180, n2);
            for j = 1:n2
                t2_val = local_theta2_range(j);
                z0_base = [deg2rad(t1_val); deg2rad(t2_val); 0; 0];
                z0_pert = z0_base; z0_pert(1) = z0_pert(1) + delta;
                [~, z_base] = fourLAdamsMoultonMethod(@(t,z) doublePendulumODE(t,z,params), [0, t_total], z0_base, options);
                [~, z_pert] = fourLAdamsMoultonMethod(@(t,z) doublePendulumODE(t,z,params), [0, t_total], z0_pert, options);
                dist_final = norm(z_base(end, :) - z_pert(end, :));
                lambda = (1/t_total) * log(dist_final / delta);
                if lambda < 0, lambda = 0; end
                col_mle(j) = lambda;
            end
            mle_matrix(:, i) = col_mle;
        end
    end
    
    % 绘图
    figure('Name', 'MLE Parameter Scan', 'Color', 'w');
    imagesc(theta1_range, theta2_range, mle_matrix);
    set(gca, 'YDir', 'normal');
    colorbar;
    colormap('jet');
    xlabel('\theta_{10} (deg)');
    ylabel('\theta_{20} (deg)');
    title(sprintf('FTLE Heatmap (T=%ds, Grid=%dx%d)', t_total, grid_res, grid_res));
    axis equal;
    xlim([-180, 180]);
    ylim([-180, 180]);
    
    fprintf('  Task 3 Completed.\n');
end

% 向量化 RK4 单步
function z_next = rk4_step(z, h, odeFunc)
    k1 = odeFunc(z);
    k2 = odeFunc(z + 0.5*h*k1);
    k3 = odeFunc(z + 0.5*h*k2);
    k4 = odeFunc(z + h*k3);
    z_next = z + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

% 向量化 ODE 函数 (支持 GPU 数组运算)
function dz = doublePendulumODE_Vectorized(z, p)
    % z is 4 x N
    th1 = z(1, :);
    th2 = z(2, :);
    w1  = z(3, :);
    w2  = z(4, :);
    
    delta_th = th1 - th2;
    cos_d = cos(delta_th);
    sin_d = sin(delta_th);
    
    % 质量矩阵元素 M = [A, B; C, D]
    A = (p.m1 + p.m2) * p.l1;
    B = p.m2 * p.l2 .* cos_d;
    C = p.l1 .* cos_d;
    D = p.l2;
    
    % 右端项 R = [R1; R2]
    R1 = -p.m2 * p.l2 .* sin_d .* w2.^2 - (p.m1 + p.m2) * p.g .* sin(th1);
    R2 = p.l1 .* sin_d .* w1.^2 - p.g .* sin(th2);
    
    % 手动解 2x2 线性方程组 (Cramer 法则)
    det_M = A .* D - B .* C;
    
    alpha1 = (R1 .* D - B .* R2) ./ det_M;
    alpha2 = (A .* R2 - C .* R1) ./ det_M;
    
    dz = [w1; w2; alpha1; alpha2];
end