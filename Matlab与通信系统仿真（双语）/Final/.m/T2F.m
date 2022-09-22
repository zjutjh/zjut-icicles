%% Fourier transform
function [f,sf] = T2F(t,st)
% input is time and the signal vectors 
% output is frequency and signal spectrum

dt = t(2) - t(1);           % sampling 
T = t(end) - t(1) + dt;     % signal duration
df = 1/T;                   % frequency accuracy
N = length(st);

% frequency[-fs/2.fs/2]
f = -N/2*df:df:(N/2-1)*df;          % Frequency sequence
sf = fft(st);
sf = T/N*fftshift(sf).*exp(-1i*2*pi*f*t(1));