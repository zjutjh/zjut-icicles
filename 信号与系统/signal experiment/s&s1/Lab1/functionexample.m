function [y]=functionexample(n,p)
% this function is used to plot the sin waveform,
% the inputs are: n, range of x-axis, p: a parameter to change the waveform
% output y is the corresponding data to the curve
y=sin(pi/p*n); 
plot(n,y)
xlabel('n');
ylabel('y');
title('It is a good example')


