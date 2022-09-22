function [T,Samp_Sig]=Sampling(t,Fs,sig) 
% Fucntion Name:Sampling 
%Input: Tb,Fs:sig OutPut:Samp_Sig 
%When you call the Function ,u input the tiem for a bit,the 
%Sampling rate and the source signal,then output the Samplint Signal.
Ts=1/Fs;
Sig=sig;
len=length(Sig);
T=0:Ts:len*t-Ts;
Samp_Sig=T; 
for i=0:1:len-1
    for j=1:1:t/Ts
        Samp_Sig(i*t/Ts+j)=Sig(i+1);
    end    
end