function [t, z] = twoLAdamsMethod(ode_func, tspan, z0, options)
% 二阶 Adams-Bashforth 显式多步法

if isfield(options, 'MaxStep'), h = options.MaxStep; else, h = (tspan(2)-tspan(1))/120000; end
step_count = ceil((tspan(2) - tspan(1)) / h);

t = linspace(tspan(1), tspan(2), step_count + 1);
num_states = numel(z0);
z = zeros(num_states, step_count + 1);
z(:, 1) = z0(:);

fVals = zeros(num_states, step_count + 1);
fVals(:, 1) = ode_func(t(1), z(:, 1));

% 使用单步 RK4 引导首个步长。
z(:, 2) = rk4Step(t(1), z(:, 1), h, ode_func);
fVals(:, 2) = ode_func(t(2), z(:, 2));

for n = 2:step_count
    z(:, n + 1) = z(:, n) + (h / 2) * (3 * fVals(:, n) - fVals(:, n - 1));
    fVals(:, n + 1) = ode_func(t(n + 1), z(:, n + 1));
end

t = t(:);
z = z.';
end

function zNext = rk4Step(t0, z0, h, ode_func)
k1 = ode_func(t0, z0);
k2 = ode_func(t0 + 0.5 * h, z0 + 0.5 * h * k1);
k3 = ode_func(t0 + 0.5 * h, z0 + 0.5 * h * k2);
k4 = ode_func(t0 + h, z0 + h * k3);
zNext = z0 + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
end
