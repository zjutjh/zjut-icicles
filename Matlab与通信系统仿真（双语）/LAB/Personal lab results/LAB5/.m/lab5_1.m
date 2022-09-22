%% Consider the following 8-PAM waveforms, where T=1.

clear;
clc;

m = [0,1,2,3,4,5,6,7];  % Equally spaced.
Am = -3.5+m;            % The amplitude of the m-th waveform.

%% Task(1)
% Give the symbol interval and bit interval.
T = 1                   % Symbol interval.
Tb = T/3                % Bit interval.

plot(Am,0,'*');grid on;title('PAM\_8 Constellation point');
for i=1:length(Am)
    text(Am(i),0,num2str(Am(i)))
end

%% Task(2)
% Give the average energy per symbol and average energy per bit.
Eav = sum(Am.^2)/8      % Average energy per symbol.
E = Eav/3               % Average energy per bit.

%% Task(3)
% Simulate the symbol error rate and bit error rate of 8-PAM when the SNR
% per bit is 0dB.

SNRindB = 0;
% simulated error rate 
[smld_err_p,smld_err_pb]=smldpe_PAM_8(SNRindB)   
