function [y]=Qfunct(x)
% [y]=Qfunct(x)
%		QFUNCT  evaluates the Q-function. 
%   		y = 1/sqrt(2*pi) * integral from x to inf of exp(-t^2/2) dt.
%     		y = (1/2) * erfc(x/sqrt(2)).
y=(1/2)*erfc(x/sqrt(2));