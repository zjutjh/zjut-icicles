% MATLAB script for Illustrative Problem 6.4.
Length=101;
Fs=10000;
W=2000;
Ts=1/Fs;
n=-(Length-1)/2:(Length-1)/2;
t=Ts*n;
h=2*W*sinc(2*W*t);        
% The rectangular windowed version follows.
N=61;
rec_windowed_h=h((Length-N)/2+1:(Length+N)/2);
% Frequency response of rec_windowed_h follows.
[rec_windowed_H,W1]=freqz(rec_windowed_h,1);
% to normalize the magnitude
rec_windowed_H_in_dB=20*log10(abs(rec_windowed_H)/abs(rec_windowed_H(1)));
% The Hanning windowed version follows.
hanning_window=hanning(N);
hanning_windowed_h=h((Length-N)/2+1:(Length+N)/2).*hanning_window.';
[hanning_windowed_H,W2]=freqz(hanning_windowed_h,1);
hanning_windowed_H_in_dB=20*log10(abs(hanning_windowed_H)/abs(hanning_windowed_H(1)));
% Plotting commands follow.
figure(1)
subplot(211)
stem(-30:30,rec_windowed_h)
subplot(212)
stem(-30:30,hanning_windowed_h)
figure(2)
subplot(211)
plot(W1/(2*pi),rec_windowed_H_in_dB)
subplot(212)
plot(W2/(2*pi),hanning_windowed_H_in_dB)