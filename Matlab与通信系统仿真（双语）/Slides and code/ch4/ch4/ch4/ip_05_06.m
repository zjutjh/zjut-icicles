% MATLAB script for Illustrative Problem 5.6. 
SNRindB1=0:1:15;
SNRindB2=0:0.1:15;
for i=1:length(SNRindB1),
   smld_err_prb1(i)=smldpe56(SNRindB1(i));   	% simulated error rate
end;
for i=1:length(SNRindB2),
   SNR=exp(SNRindB2(i)*log(10)/10);         	% signal-to-noise ratio
   theo_err_prb2(i)=Qfunct(sqrt(SNR/2));  	% theoretical error rate
end;
% Plotting commands follow.
semilogy(SNRindB1,smld_err_prb1,'*');
hold
semilogy(SNRindB2,theo_err_prb2);