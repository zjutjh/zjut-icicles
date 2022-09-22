% MATLAB script for Illustrative Problem 5.10.
SNRindB=0:2:10;	
smld_err_prb=zeros(1,length(SNRindB));
for i=1:length(SNRindB)
  % simulated error rate
  smld_err_prb(i)=smldp510(SNRindB(i));
end;
% Plotting commands follow
semilogy(SNRindB,smld_err_prb,'*');