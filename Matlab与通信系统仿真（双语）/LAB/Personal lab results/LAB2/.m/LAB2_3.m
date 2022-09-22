%% Plot the time-averaged autocorrelation and the power spectral density of x(t) 
% verify the Wiener-Khinchin theorem.

clear;
clc;

Fs = 1000;              % Sampling rate
Ts = 1/Fs;              % Sampling interval
t = 0:Ts:5;             % Sampling period
x = zeros(1,length(t)); % Input signal initiation
for i = 1:length(t)     
    if t(i)>=0&&t(i)<=2
        x(i) = t(i)*sin(100*pi*t(i));
    elseif t(i)>2&&t(i)<=5
        x(i) = (sin(pi*t(i))+2)*sin(100*pi*t(i));
    end
end
N = length(t);          % Length of t
dt = t(2)-t(1);         % Sampling period
T = t(end)-t(1)+dt;     % Signal duration

R = xcorr(x);           % average autocorrelation
R = R*Ts/T;
tau = -5:Ts:5;

figure(1);
plot(tau,R);
title('Time-averaged autocorrelation')
xlabel('t/s')
ylabel('R(t)')
grid on

[f,sf] = T2F(t,x);              % FT
P_f_spectrum = abs(sf).^2/T;    % Power spectral density(psd)
[f1,R_f] = T2F(tau,R);          % FT of the autocorrelation

figure(2)
plot(f,P_f_spectrum)
%semilogy(f,P_f_spectrum)
title('Power spectral density of x(t)')
xlabel('f/Hz')
ylabel('P\_f\_spectrum')
grid on

figure(3)
%semilogy(f1,abs(R_f))
plot(f1,abs(R_f))
title('PSD of x(t) throw Wiener-Khinchin theorem')
xlabel('f/Hz')
ylabel('abs(R\_f)')
grid on
