%%%%%%%%  This program is for sin signal sampling,   called sampling.m

function sampling(signal_f, over_f,under_f)
% signal frequency:  signal_f Hz
% to plot the continuous-time signal, we can sample the signal with quite
% high frequency. here we use "tf", you donn't change it.
%
% oversampling frequency: over_f, satifying sampling theorem
% oversampling frequency: over_t
% undersampling frequency: under_f, not satisfying sampling theorem
% undersampling frequency: under_t

% suggestions:
%  you can change parameters: 
% 1) the value of the signal frequency: signal_f
% 2) the value of the oversampling frequency: over_f
% 3) the value of the undersampling frequency: underr_f
% observe the phenomenon of the spectrum of the oversampling and
% undersampling signal.
  

% signal_f=200;      % you can change
% over_f=600;        % you can change
over_t=1/over_f;
otss=0:over_t:0.5;    
% under_f=300;       % you can change 
under_t=1/under_f;
tss=0:under_t:0.5;

tf=20*signal_f;
ot=1/tf;
t=0:ot:0.5;

x=cos(2*pi*signal_f*t);           % signal
xos=cos(2*pi*signal_f*otss);      % oversampling signal 
xs=cos(2*pi*signal_f*tss);        % undersampling signal

subplot(3,1,1);
plot(t,x);
title('The original signal');
subplot(3,1,2);
plot(t,x);
hold on;
stem(otss,xos); 
title('The oversampled signal');
subplot(3,1,3);
plot(t,x);
hold on;
stem(tss,xs,'r');
title('The undersampled signal');

[r c]=size(x);
N=r*c;
X=fftshift(fft(x,N))*ot;           % spectrum of the signal
XOS=fftshift(fft(xos,N))*over_t;   % spectrum of the over sampled signal
XS=fftshift(fft(xs,N))*under_t;      % spectrum of the under sampled signal


wo=[-N/2+1:N/2]*tf/N;               % converse to frequency Hz
wos=[-N/2+1:N/2]*over_f/N;
ws=[-N/2+1:N/2]*under_f/N;
figure;
plot(wo,abs(X));            % plot the spectrum of the signal
hold on;
plot(wos,abs(XOS),'y');     % plot the spectrum of the oversampled signal
hold on;
plot(ws,abs(XS),'r');       % plot the spectrum of the undersampled signal
title('The spectrum of the original(blue) and oversampled (yellow) and undersampled (red) signal');
xlabel('Hz')
