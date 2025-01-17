#include <bits/stdc++.h>
#include <random>

std::mt19937 rd(std::random_device{}());

int main(int argc, char **argv) {
  int n = 1e5;
  for (int i = 1; i <= n; i++) {
    int b = (rd() % 2 ? 10 : 2);
    int p = 1 << std::uniform_int_distribution<>{5, 15}(rd);
    char op = "+-*^"[rd() % 4];
  }
  return 0;
}