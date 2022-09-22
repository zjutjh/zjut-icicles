function [p]=smldPe55(snr_in_dB)
% [p]=smldPe55(snr_in_dB)
%		SMLDPE55  simulates the probability of error for the particular
%   		value of snr_in_dB, signal-to-noise ratio in dB.
E=1;
SNR=exp(snr_in_dB*log(10)/10);	   	% signal-to-noise ratio
sgma=E/sqrt(2*SNR);		   	% sigma, standard deviation of noise
N=10000;
% Generation of the binary data source follows.
for i=1:N,
   temp=rand;   	      		% a uniform random variable over (0,1)
   if (temp<0.5),
      dsource(i)=0;   	      		% With probability 1/2, source output is 0.
   else
      dsource(i)=1;	      		% With probability 1/2, source output is 1.
   end
end;
% The detection, and probability of error calculation follows.
numoferr=0;
for i=1:N,
   % the matched filter outputs
   if (dsource(i)==0),
      r=-E+gngauss(sgma);      		% if the source output is "0"
   else
      r=E+gngauss(sgma);     		% if the source output is "1"
   end;
   % Detector follows.
   if (r<0),
      decis=0;		      		% Decision is "0". 
   else
      decis=1;		      		% Decision is "1". 
   end;
   if (decis~=dsource(i)),    		% If it is an error, increase the error counter.
      numoferr=numoferr+1;
   end;
end;
p=numoferr/N;	  	      		% probability of error estimate