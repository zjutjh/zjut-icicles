%% Test name
clc;
clear all;

A = 0.5*[1 1 1 1;
        1 exp(-1i*pi/2) exp(-1i*pi) exp(-3*1i*pi/2);
        1 exp(-1i*pi) exp(-1i*2*pi) exp(-3*1i*pi);
        1 exp(-1i*3*pi/2) exp(-1i*3*pi) exp(-9*1i*pi/2);];

%% Calculate the real part, image part, amplitude, and angle of each element on the third row. (10 points)

B = A(3,:);
for i=1:4
    ans_1(i,1) = real(B(i));
    ans_1(i,2) = imag(B(i));
    ans_1(i,3) = abs(B(i));
    ans_1(i,4) = angle(B(i));
end
ans_1

%% Calculate the sum of all the entries on the second column. (10 points)

ans_2 = sum(A(:,2))

%% Verify the orthogonality of A, i.e.,AAH=AHA=I, where I is identity matrix.  (10 points)

C = A';
I_1 = A*C;
I_2 = C*A;
