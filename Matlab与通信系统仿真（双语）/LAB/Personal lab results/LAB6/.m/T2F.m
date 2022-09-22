function [f,sf]=T2F(t,st)
%input is time and the signal vectors
%output is freguency and signal spectrum

dt=t(2)-t(1);
T=t(end)-t(1)+dt;
df=1/T; %smapling rate
N=length(st);

f=-N/2*df:df:(N/2-1)*df; %频域抽样点
sf=fft(st);
sf=T/N*fftshift(sf).*exp(-j*2*pi*f*t(1));   %补偿时间移位
