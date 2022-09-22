%% Calculate the energy and power of x(t)
% in both time domain and frequency domain, and verify the equality.

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

E = sum(abs(x).^2)*dt;              %Energy of x(t) in time domain
P = sum(abs(x).^2)/N;               %Power of x(t) in frequency domain 
E_f_spectrum = sum(abs(sf).^2)*df;  %Energy of x(t) in time domain
P_f_spectrum = sum(abs(sf).^2)*df/T;%Power of x(t) in frequency domain

