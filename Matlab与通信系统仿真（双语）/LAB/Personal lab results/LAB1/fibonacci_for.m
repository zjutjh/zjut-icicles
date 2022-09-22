%% function fibonacci_for(n)
function fibo = fibonacci_for(n)
fibo = zeros(1,n);
fibo(1) = 1;        % Initialize the first two items of the sequence
fibo(2) = 1;
for k = 3:n         % Loop from the third item 
    fibo(k) = fibo(k-1) + fibo(k-2);
end
figure;
plot(fibo,'bx')
for ii = 1:1:n      % Label each point 
    text(ii,fibo(ii),['(' num2str(ii) ',' num2str(fibo(ii)) ')'])
end
title('fibonacci-for(n)')
xlabel('n')
ylabel('fibo')
legend('fibonacci-for(n)')
grid on