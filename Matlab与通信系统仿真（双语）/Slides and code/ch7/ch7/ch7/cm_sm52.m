function [p]=cm_sm52(snr_in_dB)
% [p]=cm_sm52(snr_in_dB)
%		CM_SM52  Returns the probability of error for the given
%   		value of snr_in_dB, signal-to-noise ratio in dB.
N=10000;
Eb=1;
d=1;				  
snr=10^(snr_in_dB/10);	 	  	% signal-to-noise ratio per bit
sgma=sqrt(Eb/(2*snr));	  	  	% noise variance
phi=0;
% Generation of the data source follows.
for i=1:N,	
  temp=rand;			  	% a uniform random variable between 0 and 1
  if (temp<0.5),
    dsource(i)=0;
  else
    dsource(i)=1;
  end;
end;
% detection and the probability of error calculation
numoferr=0;
for i=1:N,
  % demodulator output
  if (dsource(i)==0),
    r0c=sqrt(Eb)*cos(phi)+gngauss(sgma);
    r0s=sqrt(Eb)*sin(phi)+gngauss(sgma);
    r1c=gngauss(sgma);
    r1s=gngauss(sgma);
  else
    r0c=gngauss(sgma);
    r0s=gngauss(sgma);
    r1c=sqrt(Eb)*cos(phi)+gngauss(sgma);
    r1s=sqrt(Eb)*sin(phi)+gngauss(sgma);
  end;
  % square-law detector outputs
  r0=r0c^2+r0s^2;
  r1=r1c^2+r1s^2;
  % Decision is made next.
  if (r0>r1),
    decis=0;
  else
    decis=1;
  end;
  % If the decision is not correct the error counter is increased.
  if (decis~=dsource(i)),
    numoferr=numoferr+1;
  end;
end;
p=numoferr/(N);		  