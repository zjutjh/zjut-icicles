% MATLAB script for Illustrative Problem 7.7. 
Tb=1;
f1=1000/Tb;
f2=f1+1/Tb;
phi=pi/4;
N=5000;   				% number of samples
t=0:Tb/(N-1):Tb;
u1=cos(2*pi*f1*t);
u2=cos(2*pi*f2*t);
v1=sin(2*pi*f1*t);
v2=sin(2*pi*f2*t);
% Assuming that u1 is transmitted, the received signal r is
sgma=1;					% noise variance
for i=1:N,
  r(i)=cos(2*pi*f2*t(i)+phi)+gngauss(sgma);
end;
% The correlator outputs are computed next.
r1c=sum(r.*u1)*(Tb/(N-1));
r1s=sum(r.*v1)*(Tb/(N-1));
r2c=sum(r.*u2)*(Tb/(N-1));
r2s=sum(r.*v2)*(Tb/(N-1));

% decision variables for detector
r1=r1c^2+r1s^2
r2=r2c^2+r2s^2
% Plotting commands follow.