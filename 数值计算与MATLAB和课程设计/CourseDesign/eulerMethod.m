function [t, z] = eulerMethod(odeFunc, tspan, z0, options)
% eulerMethod 显式欧拉法

h = options.MaxStep;
step_count = ceil((tspan(2) - tspan(1)) / h);
t = linspace(tspan(1), tspan(2), step_count + 1);

num_states = numel(z0);
z = zeros(num_states, numel(t));
z(:, 1) = z0(:);

for k = 1:step_count
	z(:, k + 1) = z(:, k) + h * odeFunc(t(k), z(:, k));
end

% 转置以匹配 ode45 的输出格式 (N x 1 和 N x num_states)，省得画图报错
t = t(:);
z = z.';

end