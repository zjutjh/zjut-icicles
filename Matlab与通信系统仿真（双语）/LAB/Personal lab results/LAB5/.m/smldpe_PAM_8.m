function [p,pb]=smldpe_PAM_8(snr_in_dB)
% [p]=smldpe_PAM_8(snr_in_dB)
%           SMLDPE_PAM_8 simulates the probability of error for the given
%   		snr_in_dB, signal to noise ratio in dB.

d=1;
SNR=exp(snr_in_dB*log(10)/10);      % signal to noise ratio per bit
sgma=sqrt((5.25*d^2)/(6*SNR));      % sigma, standard deviation of nise
N=10000;                            % number of symbols being simulated

% generation of the quarternary data source follows
for i=1:N
  temp=rand;                % a uniform random variable over (0,1)
  if (temp<0.125)
    dsource(i)=0;           % with probability 1/8, source output is "000"
  elseif (temp<0.25)
    dsource(i)=1;           % with probability 1/8, source output is "001"
  elseif (temp<0.375)
    dsource(i)=2;           % with probability 1/8, source output is "010"
  elseif (temp<0.5)
    dsource(i)=3;           % with probability 1/8, source output is "011"
  elseif (temp<0.625)
    dsource(i)=4;           % with probability 1/8, source output is "100"
  elseif (temp<0.75)
    dsource(i)=5;           % with probability 1/8, source output is "101"
  elseif (temp<0.875)
    dsource(i)=6;           % with probability 1/8, source output is "110"
  else
    dsource(i)=7;           % with probability 1/8, source output is "111"
  end
end
  
% detection, and probability of error calculation
numoferr=0;
numoferr_pb=0;

for i=1:N
  % The matched filter outputs
  if (dsource(i)==0)
    r=-3.5*d+gngauss(sgma);  	% if the source output is "000"
  elseif (dsource(i)==1)
    r=-2.5*d+gngauss(sgma);    	% if the source output is "001"
  elseif (dsource(i)==2)  
    r=-1.5*d+gngauss(sgma);     % if the source output is "010"
  elseif (dsource(i)==3)  
    r=-0.5*d+gngauss(sgma);     % if the source output is "011"
  elseif (dsource(i)==4)  
    r=0.5*d+gngauss(sgma);     	% if the source output is "100"
  elseif (dsource(i)==5)  
    r=1.5*d+gngauss(sgma);     	% if the source output is "101"
  elseif (dsource(i)==6)  
    r=2.5*d+gngauss(sgma);     	% if the source output is "110"
  else
    r=3.5*d+gngauss(sgma);   	% if the source output is "111"
  end
  % detector follows
  if (r<-3*d)
    decis=0;		     		% decision is "000"
  elseif (r<-2*d)
    decis=1;		     		% decision is "001"
  elseif (r<-1*d)
    decis=2;		     		% decision is "010"
  elseif (r<0)
    decis=3;		     		% decision is "011"
  elseif (r<1*d)
    decis=4;		     		% decision is "100"
  elseif (r<2*d)
    decis=5;		     		% decision is "101"
  elseif (r<3*d)
    decis=6;		     		% decision is "110"
  else
    decis=7;		     		% decision is "111"
  end
  if (decis~=dsource(i))   		% if it is an error,
    numoferr=numoferr+1;        % increase the error counter
  end
  % Convert the judged decimal number decis into a binary number 
  % with a bit width of 3 and store it in the bit_data sequence.
  bit_data((i-1)*3+1:i*3)=dec2bin(decis,3);
end

% Convert the randomly generated decimal number dsource
% into a binary number with a bit width of 3 
% and store it in the bit_map sequence. 
for i=1:N
    bit_map((i-1)*3+1:i*3) = dec2bin(dsource(i),3);  
end

% Compare bit_map and bit_data, calculate how many bits of data are wrong.
for i=1:3*N
  if (bit_map(i)~=bit_data(i))
      numoferr_pb=numoferr_pb+1;
  end
end

p=numoferr/N;	  	     		% probability of error estimate
pb=numoferr_pb/N/3;              % probability of bit error estimate
