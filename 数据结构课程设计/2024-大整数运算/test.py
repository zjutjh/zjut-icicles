import sys, time
sys.set_int_max_str_digits(0)

start = time.time()
for _ in range(int(input())):
  line = input().split()
  b = int(line[0])
  p = int(line[1])
  x = int(line[2])
  op = line[3]
  y = int(line[4])
  if op == '+':
    ans = x + y
  elif op == '-':
    ans = x - y
  elif op == '*':
    ans = x * y
  elif op == '^':
    ans = pow(x, y) if p == 0 else pow(x, y, b ** p)
  elif op == '/':
    ans = x // y
  if p != 0:
    ans = ans % (b ** p)
  if b == 2:
    print(bin(ans)[2:])
  else:
    print(ans)
end = time.time()
print("Time: ", int((end - start) * 1000), "ms", sep="")