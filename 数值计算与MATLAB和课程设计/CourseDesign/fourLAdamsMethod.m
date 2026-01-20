function [t, z] = fourLAdamsMethod(odeFunc, tspan, z0, options)
% 四阶 Adams-Bashforth 显式多步法

if isfield(options, 'MaxStep'), h = options.MaxStep; else, h = (tspan(2)-tspan(1))/120000; end
step_count = ceil((tspan(2) - tspan(1)) / h);

t = linspace(tspan(1), tspan(2), step_count + 1);
num_states = numel(z0);
z = zeros(num_states, step_count + 1);
z(:, 1) = z0(:);

fVals = zeros(num_states, step_count + 1);
fVals(:, 1) = odeFunc(t(1), z(:, 1));

% 用 RK4 预热前三个步长，提供多步法所需历史。
for bootstrap = 1:3
    z(:, bootstrap + 1) = rk4Step(t(bootstrap), z(:, bootstrap), h, odeFunc);
    fVals(:, bootstrap + 1) = odeFunc(t(bootstrap + 1), z(:, bootstrap + 1));
end

for n = 4:step_count
    z(:, n + 1) = z(:, n) + (h / 24) * (55 * fVals(:, n) - 59 * fVals(:, n - 1) + 37 * fVals(:, n - 2) - 9 * fVals(:, n - 3));
    fVals(:, n + 1) = odeFunc(t(n + 1), z(:, n + 1));
end

t = t(:);
z = z.';
end

function zNext = rk4Step(t0, z0, h, odeFunc)
k1 = odeFunc(t0, z0);
k2 = odeFunc(t0 + 0.5 * h, z0 + 0.5 * h * k1);
k3 = odeFunc(t0 + 0.5 * h, z0 + 0.5 * h * k2);
k4 = odeFunc(t0 + h, z0 + h * k3);
zNext = z0 + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
end
