function E = calEnergy(z, params)

l1 = params.l1; l2 = params.l2;
m1 = params.m1; m2 = params.m2; g = params.g;

theta1 = z(:, 1);
theta2 = z(:, 2);
omega1 = z(:, 3);
omega2 = z(:, 4);

% 动能 T
% T = 0.5 * (m1+m2) * (l1*w1)^2 + 0.5 * m2 * (l2*w2)^2 + m2 * l1 * l2 * w1 * w2 * cos(t1-t2)
T = 0.5 * (m1 + m2) * (l1 * omega1).^2 + ...
    0.5 * m2 * (l2 * omega2).^2 + ...
    m2 * l1 * l2 .* omega1 .* omega2 .* cos(theta1 - theta2);

% 势能 V (取悬挂点 y=0，向下为负）
y1 = -l1 * cos(theta1);
y2 = y1 - l2 * cos(theta2);
V = m1 * g * y1 + m2 * g * y2;

E = T + V;
end
