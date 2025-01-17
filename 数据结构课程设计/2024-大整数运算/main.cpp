#include "big.h"
#include "io.h"
#include "io.cpp"

#include <chrono>

signed main(int argc, char **argv) {
  if (argc > 1) {
    freopen(argv[1], "r", stdin);
    if (argc > 2) {
      freopen(argv[2], "w", stdout);
    }
    int t;
    io.read(t);
    auto start = std::chrono::high_resolution_clock::now();
    while (t--) {
      int b, p;
      char op;
      io.read(b);
      io.read(p);
      Big x(b, p), y(b, p);
      io.read(x);
      io.read(op);
      io.read(y);
      if (op == '+') {
        io.write(x + y);
      } else if (op == '-') {
        io.write(x - y);
      } else if (op == '*') {
        io.write(x * y);
      } else if (op == '/') {
        io.write(x / y);
      } else if (op == '^') {
        io.write(x ^ y);
      }
      io.push('\n');
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    io.write("Time: ");
    io.write(elapsed.count());
    io.write("ms\n");
  } else {
    while (1) {
      io.write("请依次输入模底数与模指数，第一个运算数，运算符，第二个运算数:\n");
      
      int b, p;
      char op;
      io.read(b);
      io.read(p);
      Big x(b, p), y(b, p);
      io.read(x);
      io.read(op);
      io.read(y);

      auto start = std::chrono::high_resolution_clock::now();
      if (op == '+') {
        io.write(x + y);
      } else if (op == '-') {
        io.write(x - y);
      } else if (op == '*') {
        io.write(x * y);
      } else if (op == '/') {
        io.write(x / y);
      } else if (op == '^') {
        io.write(x ^ y);
      }
      io.push('\n');
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> elapsed = end - start;
      io.write("Time: ");
      io.write(elapsed.count());
      io.write("ms\n");
    }
  }
  return 0;
}