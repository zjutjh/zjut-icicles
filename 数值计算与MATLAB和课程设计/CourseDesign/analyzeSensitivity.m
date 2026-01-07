function analyzeSensitivity(ode_func, tspan, params, options)
% analyzeSensitivity 展示不同的大参数变化对运动轨迹的影响，一次做一个实验

%% 直观的初始角度实验
% 设定角度（degree）
% angles_scan = [10, 30, 45, 60, 80]; 
% figure('Name', 'Initial Condition Sensitivity (Trajectory Comparison)', 'Color', 'w', 'Position', [50, 50, 1400, 700]);

% nA = length(angles_scan);
% results1 = cell(1, nA);
% results2 = cell(1, nA);

% % 上排：固定 theta20 = 0，扫描 theta10
% parfor i = 1:nA
%     theta1_deg = anglesScan(i);
%     theta2_deg = 45;
%     local_params = params;
%     local_params.theta10 = theta1_deg;
%     local_params.theta20 = theta2_deg;
%     title_str = sprintf('\\theta_{10}=%d^\\circ, \\theta_{20}=%d^\\circ', local_params.theta10, local_params.theta20);
%     results1{i} = calculateSimulation(ode_func, tspan, local_params, options, title_str);
% end

% for i = 1:nA, plotSimulation(results1{i}, 1, i); end

% % 下排：固定 theta10 = 45，扫描 theta20
% parfor i = 1:nA
%     theta1_deg = 45;
%     theta2_deg = angles_scan(i);
%     local_params = params;
%     local_params.theta10 = theta1_deg;
%     local_params.theta20 = theta2_deg;
%     title_str = sprintf('\\theta_{10}=%d^\\circ, \\theta_{20}=%d^\\circ', local_params.theta10, theta2_deg);
%     results2{i} = calculateSimulation(ode_func, tspan, local_params, options, title_str);
% end

% for i = 1:nA, plotSimulation(results2{i}, 2, i); end

%% 直观的初始长度实验
% k1Scan = [0.01, 0.5, 1, 5, 100];
% figure('Name', 'Initial Condition Sensitivity (Trajectory Comparison)', 'Color', 'w', 'Position', [50, 50, 1400, 700]);
% nK = length(k1Scan);
% resL1 = cell(1, nK);
% resL2 = cell(1, nK);

% % 上排：固定 l2 = 1，扫描 l1
% parfor i = 1:nK
%     l2 = 1;
%     l1 = l2 * k1Scan(i);
%     local_params = params;
%     local_params.l1 = l1;
%     local_params.l2 = l2;
%     title_str = sprintf('l_1=%.2f (l_2=%.2f)', l1, l2);
%     resL1{i} = calculateSimulation(ode_func, tspan, local_params, options, title_str);
% end
% for i = 1:nK, plotSimulation(resL1{i}, 1, i); end

% % 下排：固定 l1 = 1，扫描 l2
% parfor i = 1:nK
%     l1 = 1;
%     l2 = l1 * k1Scan(i);
%     local_params = params;
%     local_params.l1 = l1;
%     local_params.l2 = l2;
%     title_str = sprintf('l_2=%.2f (l_1=%.2f)', l2, l1);
%     resL2{i} = calculateSimulation(ode_func, tspan, local_params, options, title_str);
% end
% for i = 1:nK, plotSimulation(resL2{i}, 2, i); end

%% 直观的初始质量实验
k2Scan = [0.00001, 0.005, 1, 500,10000];
figure('Name', 'Initial Condition Sensitivity (Trajectory Comparison)', 'Color', 'w', 'Position', [50, 50, 1400, 700]);
nM = length(k2Scan);
resM1 = cell(1, nM);
resM2 = cell(1, nM);

% 上排：固定 m2 = 1，扫描 m1
parfor i = 1:nM
    mass2 = 1;
    mass1 = mass2 * k2Scan(i);
    local_params = params;
    local_params.mass1 = mass1;
    local_params.mass2 = mass2;
    title_str = sprintf('m_1=%.2f (m_2=%.2f)', mass1, mass2);
    resM1{i} = calculateSimulation(ode_func, tspan, local_params, options, title_str);
end
for i = 1:nM, plotSimulation(resM1{i}, 1, i); end

% 上排：固定 m1 = 1，扫描 m2
parfor i = 1:nM
    mass1 = 1;
    mass2 = mass1 * k2Scan(i);
    local_params = params;
    local_params.mass1 = mass1;
    local_params.mass2 = mass2;
    title_str = sprintf('m_2=%.2f (m_1=%.2f)', mass2, mass1);
    resM2{i} = calculateSimulation(ode_func, tspan, local_params, options, title_str);
end
for i = 1:nM, plotSimulation(resM2{i}, 2, i); end

end

%% 公共计算函数
function res = calculateSimulation(ode_func, tspan, params, options, title_str)

    ode_funcWrapped = @(t, z) ode_func(t, z, params);
    
    % 构造初始向量
    z0 = [deg2rad(params.theta10); deg2rad(params.theta20); 0; 0];
    
    % 运行求解器 (AM4)
    [~, z] = fourLAdamsMoultonMethod(ode_funcWrapped, tspan, z0, options);
    
    % 计算末端小球 (m2) 的平面坐标 (x2, y2)
    l1 = params.l1; l2 = params.l2;
    
    x1 = l1 * sin(z(:, 1));
    y1 = -l1 * cos(z(:, 1));
    x2 = x1 + l2 * sin(z(:, 2));
    y2 = y1 - l2 * cos(z(:, 2));
    
    % 打包结果
    res.x2 = x2;
    res.y2 = y2;
    res.t1Deg = params.theta10;
    res.t2Deg = params.theta20;
    res.params = params;
    res.title_str = title_str;
end

%% 公共绘图函数
function plotSimulation(res, row, col)
    x2 = res.x2;
    y2 = res.y2;

    % 绘图位置
    subplot(2, 5, (row-1)*5 + col);
    
    % 绘制平面轨迹 XY 图
    plot(x2, y2, 'Color', '#0076a8', 'LineWidth', 1.0);
    hold on;
    
    % 标记起点
    plot(x2(1), y2(1), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 4);
    
    % 动态生成标题
    title(res.title_str, 'FontSize', 8);
    
    if col == 1
        ylabel('y (m)');
    end
    if row == 2
        xlabel('x (m)');
    end
    
    axis equal; 
    grid on;
    
    if row == 1 && col == 1
        legend('Start', 'Location', 'SouthEast', 'FontSize', 10);
    end
end