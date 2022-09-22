% An example for a script
n=-5:0.1:5;
x1=sin(pi/4*n);
x2=cos(pi/4*n);
plot(n,x1,'r');
hold on
plot(n,x2,'b');
hold off
xlabel('n');
ylabel('x');
title('It is a good figure')
legend('sin','cos')
