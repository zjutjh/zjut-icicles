function [b,fs]=filter_sys(N,fc,para)

fs=1000;
wc=2*fc/fs;
if para==0
    b=fir1(N,wc,'low');
end
if para==1
    b=fir1(N,wc,'high');
end
    
[H w]=freqz(b,1,1024,fs);
plot(w,abs(H));
figure
plot(w,angle(H))