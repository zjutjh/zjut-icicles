
%%%% this program is to obtain the frequency response of the filter

function [xm,x1m,b]=freq_respons( )
fs = input('the sampling frequency Hz:');
t=0:1/fs:2;
f = input('the frequency of input signal Hz:');
% f=ws*fs/2;
x=cos(2*pi*f*t);

N = input('the order of the filter:');
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
        disp('Invalid input. Error!');
end

[h w]=freqz(b,1,2^10,fs);

x1=filter(b,1,x);
xm=max(x);
disp('the maximum amplitude of the input signal:')
disp(xm);
x1m=max(x1);
disp('the maximum amplitude of the output signal:')
disp(x1m);

g=x1m/xm;
disp('g=');
disp(g);

figure;
subplot(2,1,1);
plot(t,x,'r');
hold on;
plot(t,x1);
title('the input and output signal');
subplot(2,1,2);
% plot(f*2/fs,g,'r');
% hold on;
plot(w,abs(h));
title('the amplitude response of the filter');
end

