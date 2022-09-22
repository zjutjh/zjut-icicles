%% Consider the following 4ASK modulation, where T=1.

clear;
clc;

%% Task(1)
% Plot the corresponding 4ASK signal.
T = 1;                  % Symbol interval.
d = 1;
Tb = T/2;               % Bit interval.
m = [1,2,3,4];          % Equally spaced.
M = 4;                  % 4ASK:M=4
A_m = (2*m-1-M)*d;      % The amplitude of the m-th waveform.
fc = 5;                 % Carrier frequency.
fs = 1000;              % Sampling frequency.
gT=sqrt(2/T);           % Transmission pulse.

% Message bits.
N = 5;
Sig = [1 1 0 0 1 0 0 0 0 1];
% Mapping.
for i=1:2:length(Sig)
    if(Sig(i)==0&&Sig(i+1)==0)
        m(fix(i/2)+1) = A_m(1);
    elseif(Sig(i)==0&&Sig(i+1)==1)
        m(fix(i/2)+1) = A_m(2);
    elseif(Sig(i)==1&&Sig(i+1)==1)
        m(fix(i/2)+1) = A_m(3);
    else
        m(fix(i/2)+1) = A_m(4);
    end
end

% Sampling.
[t,x] = Sampling(T,fs,m);
% Carrier wave.
carrier = cos(2*pi*fc*t);
% ASK modulation.
u_m=gT*x.*carrier;
% T2F.
[sf,U_m]=T2F(t,u_m);

% Plotting commands follow.
figure(1)
plot(t,u_m);
title('Time domain')
xlabel('t/s')
ylabel('u\_m(t)')
figure(2);
plot(sf,abs(U_m));
title('Frequency domain')
xlabel('f/Hz')
ylabel('U\_m')

%% Task(2)
% Give the average energy per symbol and average energy per bit.
Eav = sum(A_m.^2)/4     % Average energy per symbol.
Eb = Eav/2              % Average energy per bit.

%% Task(3)
% SER.

SNRindB=0:2:8;			
for i=1:length(SNRindB)
  % Simulated error rate .
  smld_err_prb(i)=smldpe(SNRindB(i));     
end
for i=1:length(SNRindB)
  % Signal-to-noise ratio.
  SNR_per_bit=exp(SNRindB(i)*log(10)/10);  
  % Theoretical error rate.
  theo_err_prb(i)=(3/2)*Qfunct(sqrt((4/5)*SNR_per_bit));  
end
% Plotting commands follow.
figure(3)
semilogy(SNRindB,smld_err_prb,'*');
hold
semilogy(SNRindB,theo_err_prb);
