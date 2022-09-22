intial_snr=0;
final_snr=15;
snr_step=0.25;
snr_in_dB=intial_snr:snr_step:final_snr;
for i=1:length(snr_in_dB)
    SNR=10^(snr_in_dB(i)/10);
    Pe(i)=Qfunct(sqrt(SNR));
end
semilogy(snr_in_dB,Pe);