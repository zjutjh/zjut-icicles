function [t, z] = heunMethod(ode_func, tspan, z0, options)
% Heun 方法（改进欧拉）处理一阶常微分方程组。

h = options.MaxStep;
step_count = ceil((tspan(2) - tspan(1)) / h);

t = linspace(tspan(1), tspan(2), step_count + 1);
num_states = numel(z0);
z = zeros(num_states, numel(t));
z(:, 1) = z0(:);

for k = 1:step_count
    dt = h;
    k1 = ode_func(t(k), z(:, k));
    predictor = z(:, k) + dt * k1;
    k2 = ode_func(t(k + 1), predictor);
    z(:, k + 1) = z(:, k) + dt * 0.5 * (k1 + k2);
end

t = t(:);
z = z.';
end
