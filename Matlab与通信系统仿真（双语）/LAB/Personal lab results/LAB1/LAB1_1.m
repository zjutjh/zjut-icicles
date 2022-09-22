clc;                    %Clear the contents of the command window 
clear;                  %Clear the contents of the workspace 
close;                  %Close current figure 
z1 = 3 + 4i;  	
z2 = 1 + 2i;
z3 = 2*exp(pi*1i/6);    %element in array X returns x with exponent e
z = z1 * z2 / z3;
real_z = real(z);       %return the real part of z 
imag_z = imag(z);       %return the image part of z 
abs_z = abs(z);         %return the amplitude of z 
angle_z = angle(z);     %return the angle of z 