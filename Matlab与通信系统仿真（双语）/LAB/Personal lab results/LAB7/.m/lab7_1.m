%% Consider the following 16QAM modulation, where T=1.

clear;
clc;

%% The signal.
% The corresponding 16QAM signal.
% T = 1;                  % Symbol interval.
% d = 2;
% Tb = T/4;               % Bit interval.
% m = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16];   % Equally spaced.
% M = 16;                 % QAM:M=16
% A_mc = (2*m-3)*d;       % The amplitude of the m-th waveform.
% A_ms = (3-2*m)*d;
% fc = 5;                 % Carrier frequency.
% gT=sqrt(2/T);           % Transmission pulse.
% t = 1:1:16;
% for i =1:1:16
%     u_m(t) = A_mc(t)*gT.*cos(2*pi*fc*t)+A_ms(t)*gT.*sin(2*pi*fc*t);
% end

%% Task(1) Simulate the bit error rate of 16QAM system 
% when SNR (average bit SNR) equals 5dB.
SNRindB=5;
[smld_err_p,smld_err_pb] = smldpe_QAM_16(SNRindB)    % simulated error rate
  