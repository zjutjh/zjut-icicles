function [p,pb]=smldpe_QAM_8(snr_in_dB,Es)
% [p,pb]=smldpe_QAM_8(snr_in_dB,Es)
%		smldpe_QAM_8  finds the probability of error for the given
%   		value of snr_in_dB, SNR in dB.
N=10000;
d=1;                            % min distance between symbols
Eav=Es*d^2;                     % energy per symbol
snr=10^(snr_in_dB/10);	 	  	% SNR per bit (given)
sgma=sqrt(Eav/(6*snr));	  	  	% noise variance
M=8;
% Generation of the data source follows.
for i=1:N
  temp=rand;                    % a uniform R.V. between 0 and 1
  dsource(i)=1+floor(M*temp);   % a number between 1 and 8, uniform 
end
% Mapping to the signal constellation follows.
mapping=[ sqrt(1/2)*d    sqrt(1/2)*d;
          sqrt(1/2)*d   -sqrt(1/2)*d;
         -sqrt(1/2)*d   -sqrt(1/2)*d;
         -sqrt(1/2)*d    sqrt(1/2)*d;
                  2*d            0*d;
                  0*d           -2*d;
                 -2*d            0*d;
                  0*d            2*d;];
% Mapping to the signal constellation follows as bit.
bit_mapping=[0 0 0;
             0 0 1;
             0 1 0;
             0 1 1;
             1 0 0;
             1 0 1;
             1 1 0;
             1 1 1;];
for i=1:N
  qam_sig(i,:)=mapping(dsource(i),:);
  % bit_qam_sig(i,1:3)=bit_mapping(dsource(i),:,:);
end
% received signal
for i=1:N
  [n(1) n(2)]=gngauss(sgma);
  r(i,:)=qam_sig(i,:)+n;
end
% detection and error probability calculation
numoferr=0;
numoferr_pb=0;
for i=1:N
  % Metric computation follows.
  for j=1:M
    metrics(j)=(r(i,1)-mapping(j,1))^2+(r(i,2)-mapping(j,2))^2;
  end
  [min_metric decis] = min(metrics);
  for k=1:3
    if(bit_mapping(decis,k)~=bit_mapping(dsource(i),k))
        numoferr_pb=numoferr_pb+1;
    end
  end
  if (decis~=dsource(i))
    numoferr=numoferr+1;
  end
end
p=numoferr/(N);                     % probability of error estimate
pb=numoferr_pb/(N*3);               % probability of bit error estimate