%% Test name
clc;
clear all;

% The corresponding 8QAM signal.
T = 1;                  % Symbol interval.
Tb = T/3;               % Bit interval.
m = [1,2,3,4,5,6,7,8];  % Equally spaced.
M = 8;                  % QAM:M=8
gT=sqrt(2/T);           % Transmission pulse.
A_mc = [-3,-1,1,3,-3,-1,1,3];
A_ms = [1,1,1,1,-1,-1,-1,-1];


%% Calculate the average energy per symbol and per bit of 8QAM, respectively. (10 points)

Es = sum(A_mc.^2+A_ms.^2)/8
Eb = Es/3

%% Simulate the symbol error rate of 8QAM under SNR-per-bit=0dB. (10 points)

SNRindB=0;
[smld_err_p,smld_err_pb] = smldpe_QAM_8(SNRindB,Es)    % simulated error rate

%% Plot the received constellation diagram after the 8QAM signals pass through the AWGN channel under SNR-per-bit=0dB and 10dB, respectively, explain your results. (10 points)
fc = 5;                 % Carrier frequency.
fs = 1000;              % Sampling frequency.
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
end

SNRindB_awgn_1 = 0;
snr_1 = 10^(SNRindB_awgn_1/10);	 	  % SNR per bit (given)
nc_1=sqrt(Es/(6*snr_1))*randn(7000,1);
ns_1=sqrt(Es/(6*snr_1))*randn(7000,1);
for i=1:1:7000
    Sm1_x(i) = A_mc(fix(i/1000)+1)+nc_1(i);
    Sm1_y(i) = A_mc(fix(i/1000)+1)+ns_1(i);
end
% for i=1:1:7000
%     if(x(i)==0)
%         plot(Sm1_x(i),Sm1_y(i),'r*')
%         hold on;
%     elseif(x(i)==1)
%         plot(Sm1_x(i),Sm1_y(i),'g+')
%         hold on;
%     elseif(x(i)==2)
%         plot(Sm1_x(i),Sm1_y(i),'b.')
%         hold on;
%     else
%         plot(Sm1_x(i),Sm1_y(i),'c.')
%         hold on;
%     end
%     axis square
% end

% SNRindB_awgn_2 = 10;
% snr_2 = 10^(SNRindB_awgn_2/10);	 	  % SNR per bit (given)
% nc_2=sqrt(Es/(6*snr_2))*randn(5000,1);
% ns_2=sqrt(Es/(6*snr_2))*randn(5000,1);
