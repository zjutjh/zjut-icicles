% MATLAB script for Illustrative Problem 5.5.
clear;
SNRindB1=0:1:10;
SNRindB2=0:0.1:10;
for i=1:length(SNRindB1),
  % simulated error rate
  smld_err_prb1(i)=smldpe55(SNRindB1(i));   
end;
for i=1:length(SNRindB2),
  SNR=exp(SNRindB2(i)*log(10)/10);   
  % theoretical error rate      
  theo_err_prb2(i)=Qfunct(sqrt(2*SNR));     
end;
% Plotting commands follow.
semilogy(SNRindB1,smld_err_prb1,'*');
hold
semilogy(SNRindB2,theo_err_prb2);