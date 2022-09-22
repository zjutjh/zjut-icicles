%% Calculate the symbol error rate by Monte Carlo simulation.

clear;
clc;

%% Task(1):Plot Antipodal RZ baseband signal constellation diagram.
x1=1;
y1=0;
x2=-1;
y2=0;
figure(1)
subplot(1,2,1)
plot(x1,y1,'o',x2,y2,'*')
axis('square')

%% Task(2):Suffering noise with PSD of 0.1,constellation diagram.
PSD = 0.1;
N0 = 2*PSD;
Tb = 0.5;
% Energy of a single symbol (the second half of the bipolar period is 0) 
E = 1*Tb/2;
n0=sqrt(N0/2)*randn(100,1);
n1=sqrt(N0/2)*randn(100,1);
x1=sqrt(E)+n0;
y1=0;
x2=-sqrt(E)+n1;
y2=0;
subplot(1,2,2)
plot(x1,y1,'o',x2,y2,'*')
axis('square')