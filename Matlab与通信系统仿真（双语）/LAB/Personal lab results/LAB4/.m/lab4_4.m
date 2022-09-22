%% Calculate the symbol error rate by Monte Carlo simulation.

clear;
clc;

%%Realize the optimum receiver and 
%verify the code using the antipodal baseband signal.

% Generate a single symbol.
Tb = 0.5;           % Symbol period.
fs = 1000;          % Sampling rate.
dt = 1/fs;          % Sampling interval.
N_sample = Tb*fs;   % Number of sampling points per symbol.
N = 7;              % Number of symbols.
t = 0 : dt : (N * N_sample - 1) * dt; % Sequence transmission time.
gt = [ones(1, N_sample / 2), zeros(1, N_sample / 2)]; % RZ.
% Generate baseband sequence '0 0 1 1 0 0 1'.
base1 = [0 0 1 1 0 0 1];    % 0 1 basic sequence.
s1t = [];
for i = 1 : N               % Generate original sequence.
   if base1(i)==1
       s1t = [s1t gt];
   else
       s1t = [s1t -1*gt];
   end
end

% Generate another decision signal sequence '1 1 1 1 1 1 1'.
base2 = [1 1 1 1 1 1 1];    % 0 1 decision sequence.
s2t = [];
for i = 1 : N               % Generate decision signal sequence.
   if base2(i)==1
       s2t = [s2t gt];
   else
       s2t = [s2t -1*gt];
   end
end

nt = zeros(1,N*Tb*fs);      % Noise signal which there is zero. 
rt = s1t + nt;              % Received signal.

numoferr=0;

for k = 1 : N               % Correlator.
    s1 = s1t(1:k*Tb*fs/2);  % Correlation signal sequence.
    s2 = s2t(1:k*Tb*fs/2);
    r = rt(1:k*Tb*fs/2);    % Received signal sampling at Tb.
    y1(k) = sum(r.*s1)*dt;  % Input of the detector for the kth data bit.
    y2(k) = sum(r.*s2)*dt;
    if (y1(k)>y2(k))
        decis(k)=base1(k);  % Decision is "base1". 
    else
        decis(k)=base2(k);  % Decision is "base2". 
    end
    if (decis(k)~=base1(k))% If it is an error, increase the error counter.
        numoferr=numoferr+1;
    end
end

p=numoferr/N;	  	      		% probability of error estimate.
