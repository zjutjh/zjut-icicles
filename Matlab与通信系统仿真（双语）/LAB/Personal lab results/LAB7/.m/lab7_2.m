%% Consider the following 8QAM modulation.

clear;
clc;

% The corresponding 8QAM signal.
T = 1;                  % Symbol interval.
d = 1;
Tb = T/3;               % Bit interval.
m = [1,2,3,4,5,6,7,8];  % Equally spaced.
M = 8;                  % QAM:M=8
gT=sqrt(2/T);           % Transmission pulse.

%% Task(1) Give the values of Amc and Ams for each constellation point.
% The amplitude of the m-th waveform.
A_mc = [sqrt(1/2),sqrt(1/2),-sqrt(1/2),-sqrt(1/2),2,0,-2,0]*d
A_ms = [sqrt(1/2),-sqrt(1/2),-sqrt(1/2),sqrt(1/2),0,-2,0,2]*d

%% Task(2) Plot all possible 8QAM waveforms.
fc = 5;                 % Carrier frequency.
fs = 1000;              % Sampling frequency.
figure('NumberTitle', 'off', 'Name', '8QAM Time domain waveforms');
for i=1:1:M
    m_c=A_mc(i)*gT;
    m_s=A_ms(i)*gT;
    % Sampling.
    [~,x_c] = Sampling(T,fs,m_c);
    [t,x_s] = Sampling(T,fs,m_s);
    % Carrier wave.
    carrier_c = cos(2*pi*fc*t);
    carrier_s = sin(2*pi*fc*t);
    % QAM modulation.
    u_m=x_c.*carrier_c+x_s.*carrier_s;
    % Plotting commands follow.
    subplot(2,4,i)
    plot(t,u_m);
    title(sprintf('�?d个时域波�?,i))
    grid on;
    xlabel('t/s')
    ylabel('u\_m(t)')
end
sgtitle('8QAM Time domain waveforms')

%% Task(3) Calculate the average energy per symbol and per bit.
Es = sum(A_mc.^2+A_ms.^2)/8
Eb = Es/3

%% Task(4) Simulate the symbol error rate of 8QAM when SNR-per-bit=5dB.
% when SNR (average bit SNR) equals 5dB.
SNRindB=5;
[smld_err_p,smld_err_pb] = smldpe_QAM_8(SNRindB,Es)    % simulated error rate