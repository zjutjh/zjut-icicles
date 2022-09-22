clc;
clear;
close;
z1 = 3 + 4i;
z2 = 1 + 2i;
z3 = 2*exp(pi*1i/6);
z = z1 * z2 / z3;
real_z = real(z);
imag_z = imag(z);
abs_z = abs(z);
angle_z = angle(z);
a = [z1 z2]';       % []'vector transpose 
b = [z2 z3];        % [] vector
c = b * a;          % * vector multiplication 
c1 = abs(c);
c2 = angle(c);
c(1,:) = 1;         % set the first row to '1'
c(:,2) = 1;         % set the second column to '1'
