% MATLAB script for Illustrated Problem 7.1. 
T=1;		
delta_T=T/200;		         	% sampling interval	
alpha=0.5;		         	% rolloff factor	
fc=40/T;		         	% carrier frequency
A_m=1;		  		 	% amplitude
t=-5*T+delta_T:delta_T:5*T;      	% time axis
N=length(t);
for i=1:N,
  if (abs(t(i))~=T/(2*alpha)),	 
    g_T(i) = sinc(t(i)/T)*(cos(pi*alpha*t(i)/T)/(1-4*alpha^2*t(i)^2/T^2));
  else
    g_T(i) = 0;		 		% The value of g_T is 0 at t=T/(2*alpha)
  end;				 	% and at t=-T/(2*alpha).
end;
%G_T=abs(fft(g_T));               	% spectrum of g_T
[sf,G_T]=T2F(t,g_T);

u_m=A_m*g_T.*cos(2*pi*fc*t);	 	% the modulated signal
%U_m=abs(fft(u_m));		 	% spectrum of the modulated signal
[sf,U_m]=T2F(t,u_m);

% Plotting commands follow.
figure(1);
plot(sf,abs(G_T));		 
axis([-1/T 1/T 0 max(G_T)]);
figure(2);
plot(sf,abs(U_m));