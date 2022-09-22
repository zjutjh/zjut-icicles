%% Generate random process.

clear;
clc;

%% Task(1)

% ¢ÙGenerate a random process y[n]=x1[n]+sin(x2[n]).
N = 1000;                       % The maximum value of n,sequence length
M = 50;                         % Select a sequence of length 101 = M*2+1 
Y = zeros(1,N); 
Ryav = zeros(1,2*M+1);          % 950:1050
Syav = zeros(1,2*M+1);          % 950:1050
Myav = 0;
Vyav = 0;

% Take the ensemble average over one hundred realizations.
for i = 1:2*M
    X1 = randn(1,N);         % Use 'randn(x,y)' to generate x*y matrix 
    X2 = randn(1,N);
    Y=X1+sin(X2);
    Ry = xcorr(Y,'unbiased');   % Autocorrelation of {Yn}
    Ry = Ry(N-M:N+M);           % Original Ry, m is -999 to 999(2*N-1)
    Sy = fftshift(abs(fft(Ry)));% Power spectrum of {Yn}
    Ryav=Ryav+Ry;               % 100 times
    Syav=Syav+Sy;
    Myav_time=Myav+mean(Y(1));
end
Ryav=Ryav/(2*M);                  % Time-average
Syav=Syav/(2*M);
Myav_time=Myav_time/(2*M)
%% Task(2)

% ¢ÙCalculate the mean and variance of y[n].
% Use 'mean(Y)' and 'var(Y)' to calculate the mean and variance of y[n].
Myav = mean(Y)
Vyav = var(Y)

%% Task(3)
% ¢ÙPlot the spectrum and auto-correlation of y[n].
figure(1)
subplot(121)
plot(-M:M,Ryav);
title('Auto-correction of y[n]')
xlabel('Time')
subplot(122)
plot(-0.5:0.01:0.5,Syav);
title('Spectrum of y[n]')
xlabel('Time')
