function [p,pb]=PAM8_symerr(snr_in_dB)
% [p]=PAM8_symerr(snr_in_dB)
% simulates the probability of error for the given
% snr_in_dB, signal to noise ratio in dB.
SNR=exp(snr_in_dB*log(10)/10);	   	% signal to noise ratio per bit
sgma=sqrt((5.25)/(6*SNR));	   	% sigma, standard deviation of noise
N=100000;	 		   	% number of symbols being simulated
% generation of the quarternary data source follows
numoferr=0;
numoferr_b=0;
for i=1:N
  temp=rand;   	    % a uniform random variable over (0,1)
  if (temp<0.125)
    dsource=0;   % with probability 1/8, source output is "000"
    d=[0 0 0];
  elseif (temp<0.25)
    dsource=1;   % with probability 1/8, source output is "001"
    d=[0 0 1];
  elseif (temp<0.375)
    dsource=2;  	% with probability 1/8, source output is "010"
    d=[0 1 0];
  elseif (temp<0.5)
    dsource=3;   % with probability 1/8, source output is "011"
    d=[0 1 1];
   elseif (temp<0.625)
    dsource=4;   % with probability 1/8, source output is "100"
    d=[1 0 0];
  elseif (temp<0.75)
    dsource=5;  	% with probability 1/8, source output is "101"
    d=[1 0 1];
  elseif (temp<0.875)
    dsource=6;   % with probability 1/8, source output is "110"
    d=[1 1 0];
  else
    dsource=7;   % with probability 1/8, source output is "111"  
    d=[1 1 1];
  end
% detection, and probability of error calculation
  % The matched filter outputs
  if (dsource==0),
    r=-3.5+gngauss(sgma);  		    % if the source output is "000"
  elseif (dsource==1),
    r=-2.5+gngauss(sgma);    		% if the source output is "001"
  elseif (dsource==2)  
    r=-1.5+gngauss(sgma);     		% if the source output is "010"
  elseif (dsource==3),
    r=-0.5+gngauss(sgma);    		% if the source output is "011"
  elseif (dsource==4)  
    r=0.5+gngauss(sgma);     		% if the source output is "100"
  elseif (dsource==5),
    r=1.5+gngauss(sgma);    		% if the source output is "101"
  elseif (dsource==6)  
    r=2.5+gngauss(sgma);     		% if the source output is "110"
  else
    r=3.5+gngauss(sgma);   		    % if the source output is "111"
  end;
  % detector follows
  if (r<-3),
    decis=0;		     		% decision is "000"
    decis_b=[0 0 0];
  elseif (r<-2),
    decis=1;		     		% decision is "001"
    decis_b=[0 0 1];
  elseif (r<-1),
    decis=2;		     		% decision is "010"
    decis_b=[0 1 0];
  elseif (r<0),
    decis=3;		     		% decision is "011"
    decis_b=[0 1 1];
  elseif (r<1),
    decis=4;		     		% decision is "100"
    decis_b=[1 0 0];
  elseif (r<2),
    decis=5;		     		% decision is "101"
    decis_b=[1 0 1];
  elseif (r<3),
    decis=6;		     		% decision is "110"
    decis_b=[1 1 0];
  else
    decis=7;		     		% decision is "111"
    decis_b=[1 1 1];
  end;
  if (decis~=dsource)   		% if it is an error, increase the error counter
    numoferr=numoferr+1;
  end;
  numoferr_b=numoferr_b+sum(xor(decis_b,d));
end;
p=numoferr/N % probability of error estimate
pb=numoferr_b/3/N     %probability of bit error