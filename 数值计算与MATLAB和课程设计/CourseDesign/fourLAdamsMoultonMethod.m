function [t, z] = fourLAdamsMoultonMethod(odeFunc, tspan, z0, options)
% 四阶隐式 Adams-Moulton 方法（Newton 迭代求解）

if isfield(options, 'MaxStep'), h = options.MaxStep; else, h = (tspan(2)-tspan(1))/120000; end
step_count = ceil((tspan(2) - tspan(1)) / h);

t = linspace(tspan(1), tspan(2), step_count + 1);
num_states = numel(z0);
z = zeros(num_states, step_count + 1);
z(:, 1) = z0(:);

fVals = zeros(num_states, step_count + 1);
fVals(:, 1) = odeFunc(t(1), z(:, 1));

% 用标准RK4产生足够的历史信息供多步法使用
for bootstrap = 1:3
    z(:, bootstrap + 1) = rk4Step(t(bootstrap), z(:, bootstrap), h, odeFunc);
    fVals(:, bootstrap + 1) = odeFunc(t(bootstrap + 1), z(:, bootstrap + 1));
end

tol = 1e-9;
maxIter = 8;

% 啊巴啊巴没太看懂 但是改改能用
for n = 4:step_count
    tNew = t(n+1);
    zNew = z(:, n) + (h/24)*(55*fVals(:,n) - 59*fVals(:,n-1) + 37*fVals(:,n-2) - 9*fVals(:,n-3));

    iter = 0;
    while iter < maxIter
        iter = iter + 1;
        fNew = odeFunc(tNew, zNew);
        res = zNew - z(:, n) - (h/720)*(251*fNew + 646*fVals(:,n) - 264*fVals(:,n-1) + 106*fVals(:,n-2) - 19*fVals(:,n-3));

        if norm(res, inf) < tol, break; end

        J = eye(num_states) - (h/720)*251*numericalJacobian(tNew, zNew, odeFunc, fNew);
        zNew = zNew - J \ res;

        if norm(J \ res, inf) < tol, break; end
    end

    if iter == maxIter, warning('AM2: No convergence after %d iterations', maxIter); end

    z(:, n+1) = zNew;
    fVals(:, n+1) = odeFunc(tNew, zNew);
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

function J = numericalJacobian(tPoint, zPoint, odeFunc, fAtPoint)
if nargin < 4
    fAtPoint = odeFunc(tPoint, zPoint);
end

num_states = numel(zPoint);
J = zeros(num_states);
perturbBase = sqrt(eps);

for j = 1:num_states
    step = perturbBase * max(1, abs(zPoint(j)));
    zShifted = zPoint;
    zShifted(j) = zShifted(j) + step;
    fShifted = odeFunc(tPoint, zShifted);
    J(:, j) = (fShifted - fAtPoint) / step;
end

end
