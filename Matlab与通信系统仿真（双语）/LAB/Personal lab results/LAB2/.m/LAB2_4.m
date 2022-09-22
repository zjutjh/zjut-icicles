%% Plot the quadrature component of x(t) where the carrier fc=50Hz
% plot the corresponding spectrum. 

clear;
clc;

Fs = 1000;              % Sampling rate
Ts = 1/Fs;              % Sampling interval
t = 0:Ts:5;             % Time vector
x = zeros(1,length(t)); % Input signal initiation
for i = 1:length(t)     
    if t(i)>=0&&t(i)<=2
        x(i) = t(i)*sin(100*pi*t(i));
    elseif t(i)>2&&t(i)<=5
        x(i) = (sin(pi*t(i))+2)*sin(100*pi*t(i));
    end
end
[f,sf] = T2F(t,x);      % FT
dt = t(2)-t(1);         % Sampling period
df = f(2)-f(1);         % Frequency accuracy
N = length(t);          % Length of t
T = t(end)-t(1)+dt;     % Signal duration

% Quadrature component
x_a = hilbert(x);
fc = 50;                        % Carrier fc=50Hz
x_l = x_a.*exp(-1i*2*pi*fc*t);  % Lowpass equivalent
x_i = real(x_l);                % In-phase component
x_q = imag(x_l);                % Quadrature component
[f_i,sf_i] = T2F(t,x_i);        % FT of in-phase component
[f_q,sf_q] = T2F(t,x_q);        % FT of quadrature component

figure(1)
plot(t,x_q)
title('Quadrature component of x(t)')
xlabel('t/s')
ylabel('x\_q(t)')
grid on

figure(2)
plot(f_q,abs(sf_q),'R-')
title('Spectrum of quadrature component')
xlabel('f/Hz')
ylabel('sf\_q')
grid on

figure(3)
plot(t,x_i)
title('In-phase component of x(t)')
xlabel('t/s')
ylabel('x\_i(t)')
grid on

figure(4)
plot(f_i,abs(sf_i),'R-')
title('Spectrum of in-phase component')
xlabel('f/Hz')
ylabel('sf\_i')
grid on
