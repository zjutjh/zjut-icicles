%% Sampling the Antipodal Return to Zero baseband signal.

clear;
clc;

%% Generate a single symbol 
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

%% Draw result. 
figure(1);
plot(t, st);grid on;
title('DBRZ¡®0011001¡¯');
