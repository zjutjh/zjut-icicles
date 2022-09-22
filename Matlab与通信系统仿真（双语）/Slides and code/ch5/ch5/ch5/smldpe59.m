function [p]=smldpe59(snr_in_dB)
% [p]=smldPe59(snr_in_dB)
%		SMLDPE59  simulates the error probability for the given
%   		snr_in_dB, signal-to-noise ratio in dB.
M=16;				   	% 16-ary PAM
d=1;
SNR=exp(snr_in_dB*log(10)/10);	   	% signal-to-noise ratio per bit
sgma=sqrt((85*d^2)/(8*SNR));	   	% sigma, standard deviation of noise
N=10000;	 		   	% number of symbols being simulated
% generation of the data source
for i=1:N,
   temp=rand;   	      		% a uniform random variable over (0,1)
   index=floor(M*temp);       		% The index is an integer from 0 to M-1, where
			      		% all the possible values are equally likely.
   dsource(i)=index;
end;
% detection, and probability of error calculation
numoferr=0;
for i=1:N,
  % matched filter outputs
  % (2*dsource(i)-M+1)*d is the mapping to the 16-ary constellation.
  r=(2*dsource(i)-M+1)*d+gngauss(sgma);  
  % the detector 
  if (r>(M-2)*d),
    decis=15;
  elseif (r>(M-4)*d),
    decis=14;
  elseif (r>(M-6)*d),
    decis=13;
  elseif (r>(M-8)*d),
    decis=12;
  elseif (r>(M-10)*d),
    decis=11;
  elseif (r>(M-12)*d),
    decis=10;
  elseif (r>(M-14)*d),
    decis=9;
  elseif (r>(M-16)*d),
    decis=8;
  elseif (r>(M-18)*d),
    decis=7;
  elseif (r>(M-20)*d),
    decis=6;
  elseif (r>(M-22)*d),
    decis=5;
  elseif (r>(M-24)*d),
    decis=4;
  elseif (r>(M-26)*d),
    decis=3;
  elseif (r>(M-28)*d),
    decis=2;
  elseif (r>(M-30)*d),
    decis=1;
  else
    decis=0;		    
  end;
  if (decis~=dsource(i)),   % If it is an error, increase the error counter.
    numoferr=numoferr+1;
  end;
end;
p=numoferr/N;	  	     % probability of error estimate