% MATLAB script for Illustrative Problem 7.8.
SNRindB1=0:2:15;
SNRindB2=0:0.1:15;
for i=1:length(SNRindB1),
  smld_err_prb(i)=cm_sm52(SNRindB1(i));	% simulated error rate
end;
for i=1:length(SNRindB2),
  SNR=exp(SNRindB2(i)*log(10)/10);     	% signal-to-noise ratio
  theo_err_prb(i)=(1/2)*exp(-SNR/2);	% theoretical symbol error rate
end;
% Plotting commands follow.
semilogy(SNRindB1,smld_err_prb,'*');
hold
semilogy(SNRindB2,theo_err_prb);