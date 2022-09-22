%% Consider the following 4PSK modulation.

clear;
clc;

%% Task(1)
% Plot the corresponding 4PSK signal.
T = 1;                  % Symbol interval.
Es = 1;                 % Energy of every symbol.
Tb = T/2;               % Bit interval.
m = [0,1,2,3];          % Equally spaced.
M = 4;                  % 4PSK:M=4
fc = 5;                 % Carrier frequency.
fs = 1000;              % Sampling frequency.

% Message bits.
N = 5;
Sig = [1 1 0 0 1 0 0 0 0 1];
% Mapping.
for i=1:2:length(Sig)
    if(Sig(i)==0&&Sig(i+1)==0)
        phase_m(fix(i/2)+1) = m(1);
    elseif(Sig(i)==0&&Sig(i+1)==1)
        phase_m(fix(i/2)+1) = m(2);
    elseif(Sig(i)==1&&Sig(i+1)==1)
        phase_m(fix(i/2)+1) = m(3);
    else
        phase_m(fix(i/2)+1) = m(4);
    end
end

% Sampling.
[t,x] = Sampling(T,fs,phase_m);
for i=1:1:N*T*fs
    u_m(i) =sqrt(2*Es/T)*cos(2*pi*fc*i/fs+2*pi*x(i)/4+pi/4); 
end
% T2F.
[sf,U_m]=T2F(t,u_m);

% Plotting commands follow.
figure(1)
plot(t,u_m);
title('Time domain')
xlabel('t/s')
ylabel('u\_m(t)')
figure(2);
plot(sf,abs(U_m));
title('Frequency domain')
xlabel('f/Hz')
ylabel('U\_m')

%% Task(2)
% Plot the constellation diagram.
N0 = 0.5;
nc=sqrt(N0/2)*randn(5000,1);
ns=sqrt(N0/2)*randn(5000,1);
for i=1:1:N*T*fs
    Sm_x(i) = sqrt(Es)*cos(2*pi*x(i)/4+pi/4)+nc(i);
    Sm_y(i) = sqrt(Es)*sin(2*pi*x(i)/4+pi/4)+ns(i);
end

% Plotting commands follow.
figure(3)
for i=1:1:5000
    if(x(i)==0)
        plot(Sm_x(i),Sm_y(i),'r*')
        hold on;
    elseif(x(i)==1)
        plot(Sm_x(i),Sm_y(i),'g+')
        hold on;
    elseif(x(i)==2)
        plot(Sm_x(i),Sm_y(i),'b.')
        hold on;
    else
        plot(Sm_x(i),Sm_y(i),'c.')
        hold on;
    end
    axis square
end
title('The constellation diagram of the 4PSK')

%% Task(3)
% BER and SER.

N=100000;
Eb=Es/2;
SNR = Eb/N0;
sgma=sqrt(Es/SNR/4);	  	  	% noise variance
% The signal mapping.
s00=[1 1]/sqrt(2);
s01=[-1 1]/sqrt(2);
s11=[-1 -1]/sqrt(2);
s10=[1 -1]/sqrt(2);
% Generation of the data source.
for i=1:N
  temp=rand;			  	% a uniform random variable between 0 and 1
  if (temp<0.25)		  	% With probability 1/4, source output is "00."
    dsource1(i)=0;
    dsource2(i)=0;		   
  elseif (temp<0.5)		  	% With probability 1/4, source output is "01."
    dsource1(i)=0;
    dsource2(i)=1;
  elseif (temp<0.75)	   	% With probability 1/4, source output is "11."
    dsource1(i)=1;	
    dsource2(i)=1;
  else			          	% With probability 1/4, source output is "10."
    dsource1(i)=1;
    dsource2(i)=0;
  end
end
% Detection and the probability of error calculation.
numofsymbolerror=0;
numofbiterror=0;
for i=1:N
  % The received signal at the detector, for the ith symbol, is:
  n(1)=gngauss(sgma);	  	  
  n(2)=gngauss(sgma);
  if ((dsource1(i)==0) && (dsource2(i)==0))
    r=s00+n;
  elseif ((dsource1(i)==0) && (dsource2(i)==1))
    r=s01+n;
  elseif ((dsource1(i)==1) && (dsource2(i)==1))
    r=s11+n;
  else
    r=s10+n;
  end
  % The correlation metrics are computed below.
  c00=dot(r,s00);
  c01=dot(r,s01);
  c11=dot(r,s11);
  c10=dot(r,s10);
  % The decision on the ith symbol is made next.
  c_max=max([c00 c01 c11 c10]);
  if (c00==c_max)
    decis1=0; decis2=0;
  elseif (c01==c_max)
    decis1=0; decis2=1;
  elseif (c11==c_max)
    decis1=1; decis2=1;
  else
    decis1=1; decis2=0;
  end
  % Increment the error counter, if the decision is not correct.
  symbolerror=0;
  if (decis1~=dsource1(i))
    numofbiterror=numofbiterror+1;
    symbolerror=1;
  end
  if (decis2~=dsource2(i))
    numofbiterror=numofbiterror+1;
    symbolerror=1;
  end
  if (symbolerror==1)
    numofsymbolerror = numofsymbolerror+1;
  end
end
ps=numofsymbolerror/N	          	% Since there are totally N symbols.
pb=numofbiterror/(2*N)              % Since 2N bits are transmitted.


theo_err_prb=Qfunct(sqrt(2*SNR)) 	% Theoretical bit-error rate.
theo_err_sym=theo_err_prb*2         % Theoretical symbol-error rate.