%% Simulate the band-limited channel.

clear;
clc;

Ts = 1;                 % Symbol interval.
Tb = Ts/3;              % Bit interval.
fs = 1000;              % Sampling rate.
dt = 1/fs;              % Sampling interval.
N = 10;                 % Number of symbols.
t = -N/2 : dt : N/2;    % Sequence transmission time. 
m = [0,1,2,3,4,5,6,7];  % Equally spaced.
Am = -3.5+m;            % The amplitude of the m-th waveform.

%% Task(1) Randomly generate a sequence of ten 8-PAM signals.
for i=1:length(t)
    if(fix((i-1)/fs)==((i-1)/fs))
        temp=randi(7);
        Sm(i)=Am(temp);
    else
        Sm(i)=Sm(i-1);
    end
end
[f,Sf] = T2F(t,Sm);      % TF

figure(1)
plot(t,Sm);
xlabel('t');ylabel('Amplitude '); 
title('8-PAM waveform');

figure(2)
plot(f,abs(Sf));
xlabel('f/Ts' );ylabel('');
title('8-PAM spectrum');

%% Task(2) Consider a band-limited noiseless channel.
% Generate a band-limited noiseless channel H(f)=1, |f|¡Ü3Hz.
for k = 1 : length(f)
    if abs(f(k))>3
        Hf1(k)=0;
    else
        Hf1(k)=Ts;
    end
end
for k = 1 : length(f)
    if abs(f(k))>300
        Hf2(k)=0;
    else
        Hf2(k)=Ts;
    end
end

% Plot the band-limited noiseless channel.
figure(3)
plot(f,Hf1,f,Hf2);
grid on;
xlabel ('f/Ts'); ylabel('Noiseless channel spectrum');
axis([ - 400 400 0  1.2]);
legend('cut-off f=3','cut-off f=300');


% Passing through this channel.
value_t=t(1);
Xf1 = Sf.*Hf1;
Xf1 = Xf1.*exp(1i*2*pi*f*value_t);
[t,Xt1] = F2T(f,Xf1);      % FT

Xf2 = Sf.*Hf2;
Xf2 = Xf2.*exp(1i*2*pi*f*value_t);
[t,Xt2] = F2T(f,Xf2);      % FT

figure(4)
subplot(121)
plot(f,abs(Xf1));
xlabel('f/Ts');ylabel(''); 
axis([ -5 5 0 6]);
title('8-PAM waveform passomg noiseless channel spectrum');
legend('cut-off f=3');
subplot(122)
plot(f,abs(Xf2));
xlabel('f/Ts');ylabel(''); 
axis([ -50 50 0 6]);
title('8-PAM waveform passomg noiseless channel spectrum');
legend('cut-off f=300');

figure(5)
subplot(121)
plot(t,real(Xt1));
axis([ 0 N -4 4]);
xlabel('t');ylabel('Amplitude'); 
title('8-PAM waveform passing noiseless channel waveform');
legend('cut-off f=3');

subplot(122)
plot(t,real(Xt2));
axis([ 0 N -4 4]);
xlabel('t');ylabel('Amplitude'); 
title('8-PAM waveform passing noiseless channel waveform');
legend('cut-off f=300');