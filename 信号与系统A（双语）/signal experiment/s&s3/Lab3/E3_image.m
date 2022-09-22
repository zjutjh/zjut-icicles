
function E3_image()
clc;
clear;
inp2=imread('Lena.tif');
myfilter=fir1(20,0.4,'high');
myfilter2=ftrans2(myfilter);
outp2=imfilter(inp2,myfilter2);
subplot(1,2,1);
imshow(inp2);
title('The original image');
hold on;
subplot(1,2,2);
imshow(outp2);
title('The filtered image');