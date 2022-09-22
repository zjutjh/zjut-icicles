
function E3_rect_pulse( )
clc;
clear;
tarray=-10:0.1:10;
T=2;
plot(tarray, rectpuls(tarray,T));   % to generate the rectangular pulse
axis([-10 10 -0.2 1.2]);
title('The rectangular pulse');

syms x f;
P=int(exp(-j*2*pi*f*x), x, -T/2, T/2);  % integral
ezplot(P,-2,2);
title('The FT of rectangular pulse');