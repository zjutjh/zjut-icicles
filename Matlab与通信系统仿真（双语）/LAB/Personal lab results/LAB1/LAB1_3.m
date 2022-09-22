clc;
clear;
close;
x = 0:pi/50:2*pi;   % from 0 to 2*pi,step is pi/50
y1 = sin(x);
y2 = cos(x);
y3 = log(x);

figure(1);          % Create a new picture 

subplot(2,2,1)      
% subplot(2,2,1) means this figure has 2 rows and 2 columns in total,
% and this subfigure is the first one of those subfigures.
plot(x,y1)
title('sin(x)')     % set the title of this subfigure
xlabel('x')         % set the x-axis label of this subfigure
ylabel('sin(x)')    % set the x-axis label of this subfigure
legend('sin(x)')    % set the legend of this subfigure
grid on             % set gridlines

subplot(2,2,2)
plot(x,y2)
title('cos(x)')
xlabel('x')
ylabel('cos(x)')
legend('cos(x)')
grid on

subplot(2,2,3)
plot(x,y3)
title('log(x)')
xlabel('x')
ylabel('log(x)')
legend('log(x)')
grid on

subplot(2,2,4)

plot(x,y1,'bh')
xlabel('x')
ylabel('sin(x)')

hold on                 % Continue to draw on this subfigure
plot(x,y2,'gP')
xlabel('x')
ylabel('cos(x)')

hold on
plot(x,y3,'r>')
xlabel('x')
ylabel('log(x)')

title('sin&&cos&&log')
legend('sin(x)','cos(x)','log(x)')
grid on