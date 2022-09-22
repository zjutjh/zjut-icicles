%% Generate the following signal
% where the duration is [0,5], sampling rate is 1000Hz. 
% Plot the signal and its magnitude spectrum. 
% x(t)=t*sin(100*pi*t), for t within [0,2]
% x(t)=(sin(pi*t)+2)*sin(100*pi*t), otherwise

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

figure(1)
plot(t,x,'b-')
title('Original signal')
xlabel('t/s')
ylabel('x(t)')
grid on

figure(2)
plot(f,abs(sf),'R--')
title('Spectrum')
xlabel('f/Hz')
ylabel('sf')
grid on
