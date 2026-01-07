function dzdt = doublePendulumODE(~, z, params)
% doublePendulumODE 复合双摆的一阶微分方程组
% 参数：时间(疑似用不上) 状态向量 [theta1; theta2; omega1; omega2] 几何参数 l1, l2, m1, m2, g
% dzdt: 状态导数 [omega1; omega2; alpha1; alpha2]

l1 = params.l1; l2 = params.l2;
m1 = params.m1; m2 = params.m2; g = params.g;

theta1 = z(1); theta2 = z(2);
omega1 = z(3); omega2 = z(4);

% M矩阵 和 右侧不知道是啥的矩阵（力吗？）
% $$\begin{bmatrix}
% (m_1 + m_2) l_1 & m_2 l_2 \cos(\theta_1 - \theta_2) \\
% l_1 \cos(\theta_1 - \theta_2) & l_2
% \end{bmatrix}
% \begin{bmatrix} \ddot{\theta_1} \\ \ddot{\theta_2} \end{bmatrix}
% =
% \begin{bmatrix}
% - m_2 l_2 \sin(\theta_1 - \theta_2) \dot{\theta_2}^2 - (m_1 + m_2) g \sin\theta_1 \\
% l_1 \sin(\theta_1 - \theta_2) \dot{\theta_1}^2 - g \sin\theta_2
% \end{bmatrix}$$
mass_matrix = [ (m1 + m2) * l1, m2 * l2 * cos(theta1 - theta2);
               l1 * cos(theta1 - theta2), l2 ];

right_side = [ -m2 * l2 * sin(theta1 - theta2) * omega2^2 - (m1 + m2) * g * sin(theta1);
            l1 * sin(theta1 - theta2) * omega1^2 - g * sin(theta2) ];

% 求解角加速度，直接除
alpha = mass_matrix \ right_side;
dzdt = [omega1; omega2; alpha(1); alpha(2)];
end
