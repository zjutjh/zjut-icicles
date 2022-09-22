%% Consider the following 3FSK modulation.

clear;
clc;

%% Task(1) Plot all possible 3FSK waveforms in both time and frequency domain.
T = 1;                  % Symbol interval.
Es = 1;                 % Energy per symbol.
fc = 10;                % Carrier frequency.
fs = 1000;              % Sampling frequency.
delta_f = 2;
M = 3;
amplitude = square(2*Es/T);
figure('NumberTitle', 'off', 'Name', '3FSK waveforms');
for m=1:1:M
    % Sampling.
    [t,x] = Sampling(T,fs,amplitude);
    % Carrier wave.
    carrier = cos(2*pi*fc*t+2*pi*m*delta_f*t);
    % QAM modulation.
    u_m=x.*carrier;
    % T2F.
    [sf,U_m]=T2F(t,u_m);
    % Plotting commands follow.
    subplot(2,3,m)
    plot(t,u_m);
    grid on;
    title(sprintf('第%d个时域波形',m))
    xlabel('t/s')
    ylabel('u\_m(t)')
    subplot(2,3,m+3)
    plot(sf,abs(U_m));
    grid on;
    title(sprintf('第%d个频域波形',m))
    axis([-30,30,0,0.6])
    xlabel('f/Hz')
    ylabel('U\_m')
end
% sgtitle('3FSK waveforms')