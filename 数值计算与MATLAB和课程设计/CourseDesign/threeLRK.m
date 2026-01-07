function [t, z] = threeLRK(ode_func, tspan, z0, options)
% 经典三阶 Runge-Kutta 方法（c2=1/2, c3=1）

h = options.MaxStep;
step_count = ceil((tspan(2) - tspan(1)) / h);

t = linspace(tspan(1), tspan(2), step_count + 1);
numStates = numel(z0);
z = zeros(numStates, numel(t));
z(:, 1) = z0(:);

for k = 1:step_count
    dt = h;
    k1 = ode_func(t(k), z(:, k));
    k2 = ode_func(t(k) + 0.5 * dt, z(:, k) + 0.5 * dt * k1);
    stageArg = z(:, k) + dt * (-k1 + 2 * k2);
    k3 = ode_func(t(k + 1), stageArg);
    z(:, k + 1) = z(:, k) + dt * (k1 + 4 * k2 + k3) / 6;
end

t = t(:);
z = z.';
end
