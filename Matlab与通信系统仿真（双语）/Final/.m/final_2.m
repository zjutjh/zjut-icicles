%% Test name
clc;
clear all;

% Initialization.
t0 = 4;                 % Signal duration
fs = 100;              % Sampling rate
Ts = 1/fs;              % Sampling interval
fc = 10;                % Carrier rate
t = 0:Ts:t0;            % Time vector
N = length(t)-1;        % Length of t
dt = t(2)-t(1);
Ac = 1;

%% Plot the signal in the time domain and its magnitude spectrum, explain the resulted spectrum. (20 points)

m = cos(2*pi*t)+sin(pi*t);
s = Ac*m.*cos(2*pi*fc*t);
[f,Sf] = T2F(t,s); 

figure('NumberTitle', 'off', 'Name', 'Signal of final exam2_1');
subplot(121)
plot(t,s);
grid on;
xlabel('Time(s)');ylabel('Amplitude');
title('Time domain waveform of s');
legend('T-domain');

subplot(122)
plot(f,abs(Sf));
grid on;
xlabel('Frequency(Hz)');ylabel('Amplitude');
axis([-30 30 0 1.2 ]);
title('Frequency spectrum of Sf');
legend('F-domain');

suptitle('Signal of final exam2\_1');

%% Calculate the energy and power of the signal from 0s to 4s, in time domain. (10 points)

E_time = sum(abs(s).^2)*dt
P_time = sum(abs(s).^2)/N

%% The signal is passing through a noiseless bandlimited ¡®channel¡¯, whose response is H(f)=1, |f|¡Ü2Hz and H(f)=0, otherwise. Plot the resulted signal in the time domain. Compare with the original signal and explain your results (10 points)	

for k = 1 : length(f)
    if abs(f(k))>2
        Hf(k)=0;
    else
        Hf(k)=1;
    end
end
Xf = Sf.*Hf;
Xf = Xf.*exp(1i*2*pi*f*t(1));
[t,Xt] = F2T(f,Xf);      % FT

figure('NumberTitle', 'off', 'Name', 'Signal of final exam2_3');
subplot(121)
plot(t,s);
grid on;
% axis([ 0 N -4 4]);
xlabel('t(s)');ylabel('Amplitude'); 
title('Original signal in the time domain');
legend('Time-domain');

subplot(122)
plot(t,Xt);
grid on;
% axis([ 0 N -4 4]);
xlabel('t(s)');ylabel('Amplitude'); 
title('Passing through a noiseless bandlimited channel');
legend('Signal>2Hz has been filtered');

suptitle('Signal of final exam2\_3');
