
%%%%%%%%%%%%%This program is speech filtering, called E3_speech_filt.m

function [S_in, S_out]=E3_speech_Filt( )

%%%%%% N: the order of the filter;
%%%%%% f_c:  the cutoff frequency of the filter in the range betwwen o and 1 ;
%%%%%% para: to choose the filter type: para=0, lowpass filter, para=1,
%%%%%% high pass filter

% obtain the filter, and plot the amplitude and phase response
fs=16000;
N =input('the order of the filter:');
para=input('select the filter type:0---low pass\1---high pass:\2---bandpass\3---stoppass:');
switch para
    case 0
        fc= input('the cutoff frequency of the filter Hz:');
        wc=2*fc/fs;
        b=fir1(N,wc,'low');
    case 1
        fc= input('the cutoff frequency of the filter Hz:');
        wc=2*fc/fs;
        b=fir1(N,wc,'high');
    case 2
        fc= input('the central frequency of the filter Hz:');
        fw= input('the passband width of the filter Hz:');
        w1=2*(fc-fw/2)/fs;
        w2=2*(fc+fw/2)/fs;
        b=fir1(N,[w1 w2]);
    case 3
        fc= input('the central frequency of the filter Hz:');
        fw= input('the stopband width of the filter Hz:');
        w1=2*(fc-fw/2)/fs;
        w2=2*(fc+fw/2)/fs;
        b=fir1(N,[w1 w2],'stop');
    otherwise
        disp('No such filter. Use the configuration at last time');
end

[H,w]=freqz(b,1,2^10,fs);
subplot(2,1,1);
plot(w,abs(H));
title('The amplitude response of the filter');
subplot(2,1,2);
plot(w,angle(H));
title('The phase response of the filter');
% to filter the speech
fs=16000;
ts=1/fs;
y=audioread('f1_16.wav');            % here the filename of the original speech
S_in=audioread('f1_16noisespeech.wav');   % the noise speech
[r c]=size(S_in);                   % length of the speech
t=0:r-1;
t=t*ts;                          % compute time axis
S_out=filter(b,1,S_in);
NL=r*c; L=fix(NL/2);
f=0:1:L-1;  
farray=f*fs/NL;
%%%%%%% to obtain the spectrum of the original speech
y1=fft(y);
y1=y1(1:L);
y2=abs(y1);

%%%%%%% to obtain the spectrum of the noised speech
X1=fft(S_in);
X1=X1(1:L);
X2=abs(X1);

%%%%%%% to obtain the spectrum of the filtered speech
Y1=fft(S_out);
% Y2=fftshift(Y1);
Y1=Y1(1:L);
Y2=abs(Y1);

figure;
subplot(3,1,1);
plot(t,y);
title('Original Speech');
axis([t(1) t(r) -0.5 0.5]);
subplot(3,1,2);
plot(t,S_in);
title('Noised Speech');
axis([t(1) t(r) -0.5 0.5]);
subplot(3,1,3);
plot(t,S_out);
title('Filtered Speech');
axis([t(1) t(r) -0.5 0.5]);

figure;
subplot(3,1,1);
plot(farray,y2);
title('The spectrum of the original speech');
xlabel('Hz');
subplot(3,1,2);
plot(farray,X2,'r');
title('The spectrum of the noised speech');
xlabel('Hz');
hold on;
subplot(3,1,3);
plot(farray,Y2,'b');
title('The spectrum of the filtered speech');
xlabel('Hz');





