%% Validation of band-limited signal design with zero ISI .

clear;
clc;

%% Generate a single symbol 
Tb1 = 1;                    % Symbol period.
Tb2 = 0.5;
fs = 1000;                  % Sampling rate.
dt = 1/fs;                  % Sampling interval.
N_sample1 = Tb1*fs;         % Number of sampling points per symbol.
N_sample2 = Tb2*fs;
N = 8;                      % Number of symbols.
t1 = 0 : dt : (N * N_sample1 - 1) * dt; % Sequence transmission time.
t2 = 0 : dt : (N * N_sample2 - 1) * dt;
gt1 = [ones(1, N_sample1)];
gt2 = [ones(1, N_sample2)];
base = [-1 -1 1 -1 1 -1 1 -1]; % 0 1 basic sequence. 
st1 = [];st2 = [];

% Generate sequence.
for i = 1 : N
   if base(i)==1
       st1 = [st1 gt1];
   else
       st1 = [st1 -1*gt1];
   end
end
for i = 1 : N
   if base(i)==1
       st2 = [st2 gt2];
   else
       st2 = [st2 -1*gt2];
   end
end

figure(1);
plot(t1, st1);grid on;
title('Raw binary sequence(no ISI Tb=1)');
figure(2);
plot(t2, st2);grid on;
title('Raw binary sequence(ISI Tb=0.5)');

% Ideal rectangular low-pass filter.
h = sinc(t1);            
figure(3);
plot(t1,h);grid on;
title('Ideal rectangular low-pass filter')

figure(4);
st_NoISI = conv(h,st1)*dt;
st_NoISI = st_NoISI(1:length(t1));
plot(t1,st_NoISI);grid on;hold on;
n=0.5:1:7.5;
stem(n,base);title('No ISI');

figure(5);
st_ISI = conv(h,st2)*dt;
st_ISI = st_ISI(1:length(t2));
plot(t2,st_ISI);grid on;hold on;
n=0.25:0.5:3.75;
stem(n,base);title('ISI');
