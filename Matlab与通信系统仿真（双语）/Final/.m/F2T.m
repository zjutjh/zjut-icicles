%% Inverse Fourier Transform
function [t,st] = F2T(f,sf)
% output is time ande signal sequence
% input is frequency and signal spectrum

df = f(2) - f(1);
Fmx = f(end) - f(1) + df;
dt = 1/Fmx;                 % sampling rate
N = length(sf);
T = N*dt;

t = 0:dt:T-dt;              % time sequence
sff = ifftshift(sf);
st = Fmx*ifft(sff);
