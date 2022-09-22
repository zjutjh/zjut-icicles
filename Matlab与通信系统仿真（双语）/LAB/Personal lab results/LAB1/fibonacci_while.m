%% function fibonacci_while(n)
function fibo = fibonacci_while(n)
fibo = zeros(n, 1);
fibo(1) = 1;        % Initialize the first two items of the sequence
fibo(2) = 1;
k = 3;              % Loop from the third item 
while k <= n
    fibo(k) = fibo(k-1) + fibo(k-2);
    k = k + 1;
end
figure;
plot(fibo,'rd')
for ii = 1:1:n      % Label each point 
    text(ii,fibo(ii),['(' num2str(ii) ',' num2str(fibo(ii)) ')'])
end
title('fibonacci-while(n)')
xlabel('n')
ylabel('fibo')
legend('fibonacci-while(n)')
grid on