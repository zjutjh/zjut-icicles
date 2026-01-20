function [t, z] = fourLRK(ode_func, tspan, z0, options)
% 四阶经典 Runge-Kutta 方法（RK4）

if isfield(options, 'MaxStep'), h = options.MaxStep; else, h = (tspan(2)-tspan(1))/120000; end
step_count = ceil((tspan(2) - tspan(1)) / h);

t = linspace(tspan(1), tspan(2), step_count + 1);
num_states = numel(z0);
z = zeros(num_states, numel(t));
z(:, 1) = z0(:);

for k = 1:step_count
    dt = h;
    k1 = ode_func(t(k), z(:, k));
    k2 = ode_func(t(k) + 0.5 * dt, z(:, k) + 0.5 * dt * k1);
    k3 = ode_func(t(k) + 0.5 * dt, z(:, k) + 0.5 * dt * k2);
    k4 = ode_func(t(k + 1), z(:, k) + dt * k3);
    z(:, k + 1) = z(:, k) + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
end

t = t(:);
z = z.';
end
