clear; clc;
% 载入初始条件（几何参数和初始静态条件）
run('initalCondition.m');
params = struct('l1', l1, 'l2', l2, 'm1', m1, 'm2', m2, 'g', g, 'theta10', theta10, 'theta20', theta20);
z0 = [deg2rad(theta10); deg2rad(theta20); 0; 0];

% 积分设置：时间区域、各求解器自己还需要的参数
tspan = [0, 20];
options = odeset('RelTol', 1e-10, 'AbsTol', 1e-12, 'MaxStep', 0.0005);

% 各种算法生成参考解，如果想看动画在这里取消掉对应的算法的注释
% [t, z] = ode45(@(tt, zz) doublePendulumODE(tt, zz, params), tspan, z0, options);
% [t, z] = eulerMethod(@(tt, zz) doublePendulumODE(tt, zz, params), tspan, z0, options);
% [t, z] = heunMethod(@(tt, zz) doublePendulumODE(tt, zz, params), tspan, z0, options);
% [t, z] = threeLRK(@(tt, zz) doublePendulumODE(tt, zz, params), tspan, z0, options);
% [t, z] = fourLRK(@(tt, zz) doublePendulumODE(tt, zz, params), tspan, z0, options);
% [t, z] = twoLAdamsMethod(@(tt, zz) doublePendulumODE(tt, zz, params), tspan, z0, options);
% [t, z] = fourLAdamsMethod(@(tt, zz) doublePendulumODE(tt, zz, params), tspan, z0, options);
% [t, z] = fourLAdamsMoultonMethod(@(tt, zz) doublePendulumODE(tt, zz, params), tspan, z0, options);

%% 精度对比
solversList = {
    @eulerMethod, 'Euler';
    @heunMethod, 'Heun';
    @threeLRK, 'RK3';
    @fourLRK, 'RK4';
    @twoLAdamsMethod, 'Adams2';
    @fourLAdamsMethod, 'Adams4';
    @fourLAdamsMoultonMethod, 'AdamsMoulton4'
};
if ~isempty(solversList), compareSolvers(@doublePendulumODE, tspan, z0, params, solversList, options); end

%% 误差分析
% 在solversList 中取消注释对应部分
if ~isempty(solversList), analyzeError(@doublePendulumODE, [0, 100], z0, params, solversList); end

%% 敏感性分析
% analyzeSensitivity(@doublePendulumODE, tspan, params, options);

% 【想看动画或者看下面的轨迹请取消注释】计算两端点在平面中的轨迹坐标，并将所有状态合并到一个矩阵中 [t, x1, y1, x2, y2]^T 便于后续研究
% x1 = l1 * sin(z(:, 1));
% y1 = -l1 * cos(z(:, 1));
% x2 = x1 + l2 * sin(z(:, 2));
% y2 = y1 - l2 * cos(z(:, 2));
% trajectory = [t, x1, y1, x2, y2].';

% 可视化角度响应（转为度便于解读）。
% figure('Name', 'Double Pendulum Trajectory', 'Color', 'w');
% plot(t, rad2deg(z(:, 1)), 'LineWidth', 1.5, 'DisplayName', '\theta_1');
% hold on;
% plot(t, rad2deg(z(:, 2)), 'LineWidth', 1.5, 'DisplayName', '\theta_2');
% hold off;
% legend('Location', 'best');
% xlabel('时间 / s');
% ylabel('角度 / °');
% title('复合双摆在 60 s 内的角度响应');
% grid on;

% 相图展示。
% figure('Name', 'Phase Portraits', 'Color', 'w');
% subplot(1, 2, 1);
% plot(rad2deg(z(:, 1)), rad2deg(z(:, 3)), 'LineWidth', 1.2);
% xlabel('\theta_1 / °');
% ylabel('\omega_1 / (°/s)');
% title('摆杆 1 相图');
% grid on;

% subplot(1, 2, 2);
% plot(rad2deg(z(:, 2)), rad2deg(z(:, 4)), 'LineWidth', 1.2);
% xlabel('\theta_2 / °');
% ylabel('\omega_2 / (°/s)');
% title('摆杆 2 相图');
% grid on;

% 平面轨迹图。
% figure('Name', 'Planar Motion', 'Color', 'w');
% plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', '铰点');
% hold on;
% plot(x1, y1, 'LineWidth', 1.2, 'DisplayName', '小球 1 轨迹');
% plot(x2, y2, 'LineWidth', 1.2, 'DisplayName', '小球 2 轨迹');
% scatter(x1(1), y1(1), 40, 'filled', 'MarkerFaceColor', [0 0.45 0.74], 'DisplayName', '起点 1');
% scatter(x2(1), y2(1), 40, 'filled', 'MarkerFaceColor', [0.85 0.33 0.1], 'DisplayName', '起点 2');
% axis equal;
% xlabel('x / m');
% ylabel('y / m');
% title('0-60 s 内端点轨迹');
% legend('Location', 'bestoutside');
% grid on;
% hold off;

%% 动画演示：展示 0-60 s 内摆杆与小球的实时运动
% 这部分的代码由QwenCoder编写，实在是没空仔细查代码了
% figure('Name', 'Pendulum Animation', 'Color', 'w');
% axis equal;
% hold on;
% plot(0, 0, 'ko', 'MarkerFaceColor', 'k');
% axisPadding = 1.1 * max([l1 + l2; abs(x1(:)); abs(y1(:))]);
% axis([-axisPadding, axisPadding, -axisPadding, axisPadding]);
% xlabel('x / m');
% ylabel('y / m');
% title('复合双摆 0-60 s 动画');
% grid on;

% rod1 = plot([0, trajectory(2,1)], [0, trajectory(3,1)], 'LineWidth', 2, 'Color', [0 0.45 0.74]);
% rod2 = plot([trajectory(2,1), trajectory(4,1)], [trajectory(3,1), trajectory(5,1)], 'LineWidth', 2, 'Color', [0.85 0.33 0.1]);
% mass1 = plot(trajectory(2,1), trajectory(3,1), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0 0.45 0.74], 'MarkerEdgeColor', 'none');
% mass2 = plot(trajectory(4,1), trajectory(5,1), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.85 0.33 0.1], 'MarkerEdgeColor', 'none');

% frameSkip = max(1, round(numel(t) / 1800));
% % 瞎选的参数 反正能看效果
% for idx = 1:frameSkip:numel(t)
%     current_frame = trajectory(:, idx);
%     cx1 = current_frame(2); cy1 = current_frame(3);
%     cx2 = current_frame(4); cy2 = current_frame(5);

%     set(rod1, 'XData', [0, cx1], 'YData', [0, cy1]);
%     set(rod2, 'XData', [cx1, cx2], 'YData', [cy1, cy2]);
%     set(mass1, 'XData', cx1, 'YData', cy1);
%     set(mass2, 'XData', cx2, 'YData', cy2);
%     drawnow;
% end
