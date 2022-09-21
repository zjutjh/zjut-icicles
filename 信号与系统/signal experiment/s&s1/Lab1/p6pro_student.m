function [y]=p6pro_student(x,n,alpha)
N=length(n);  
y=zeros(N,1); % initialize output vector y

y(1)=           % the start point of the output

for loop=2:N     % use for loop to calculate the rest of y
    y(loop)=   ; % each loop, we calculate one element in y, that is y(loop)
end
figure
plot(n,x)
title('x')
figure
plot(n,y)
title('y')