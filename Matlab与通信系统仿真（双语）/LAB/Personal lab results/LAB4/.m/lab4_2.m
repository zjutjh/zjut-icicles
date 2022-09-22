%% Sampling the Antipodal Return to Zero baseband signal.

clear;
clc;

%% Task
Tb = 0.5;           % Symbol period.
fs = 1000;          % Sampling rate.
dt = 1/fs;          % Sampling interval.
N_sample = Tb*fs;   % Number of sampling points per symbol.
N = 7;              % Number of symbols.
t = 0 : dt : (N * N_sample - 1) * dt; % Sequence transmission time. 
gt = [ones(1, N_sample / 2), zeros(1, N_sample / 2)]; % RZ.

%% Generate sequence '0 0 1 1 0 0 1'.
base = [0 0 1 1 0 0 1]; % 0 1 basic sequence. 
st = [];
for i = 1 : N           % Generate sequence.
   if base(i)==1
       st = [st gt];
   else
       st = [st -1*gt];
   end
end

% explaination:At the very beginning, 
% we thought that we were given a snr_in_db, 
% and felt that there was a problem with the problem expression, 
% and then we realized that we were going 
% to use the sequence of task1 to calculate E. 

%snr_in_dB=5.5;                  % Give the SNR (i.e., E/N0) in dB.
%SNR=exp(snr_in_dB*log(10)/10);	% Signal-to-noise ratio.
%E = SNR*N0;


PSD = 0.1;
N0 = 2*PSD;
E = sum(abs(st).^2)*dt/7;
SNR = E/N0;
sgma=E/sqrt(2*SNR);             % Sigma, standard deviation of noise.
N=1000;

% Generation of the binary data source follows.
for i=1:N
   temp=rand;                   % A uniform random variable over (0,1).
   if (temp<0.5)
      dsource(i)=0;             % With probability 1/2, source output is 0.
   else
      dsource(i)=1;             % With probability 1/2, source output is 1.
   end
end
% The detection, and probability of error calculation follows.
numoferr=0;
for i=1:N
   % The matched filter outputs.
   if (dsource(i)==0)
      r=-E+gngauss(sgma);       % if the source output is "0".
   else
      r=E+gngauss(sgma);        % if the source output is "1".
   end
   % Detector follows.
   if (r<0)
      decis(i)=0;               % Decision is "0". 
   else
      decis(i)=1;               % Decision is "1". 
   end
   % If it is an error, increase the error counter.
   if (decis(i)~=dsource(i))
      numoferr=numoferr+1;  
   end
end
p=numoferr/N;	  	      		% probability of error estimate