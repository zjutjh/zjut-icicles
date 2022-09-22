% MATLAB script for Illustrative Problem 7.3.
SNRindB1=0:2:10;
SNRindB2=0:0.1:10;
for i=1:length(SNRindB1),
  [pb,ps]=cm_sm32(SNRindB1(i));    	% simulated bit and symbol error rates
  smld_bit_err_prb(i)=pb; 
  smld_symbol_err_prb(i)=ps;
end;
for i=1:length(SNRindB2),
  SNR=exp(SNRindB2(i)*log(10)/10);     	% signal-to-noise ratio
  theo_err_prb(i)=Qfunct(sqrt(2*SNR)); 	% theoretical bit-error rate
end;
% Plotting commands follow
semilogy(SNRindB1,smld_bit_err_prb,'*');
hold
semilogy(SNRindB1,smld_symbol_err_prb,'o');
semilogy(SNRindB2,theo_err_prb);