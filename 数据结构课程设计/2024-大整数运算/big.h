#ifndef BIG_H
#define BIG_H

#include "complex.h"
#include "complex.cpp"
#include "list.h"

using u32 = unsigned int;
using i64 = long long;
using u64 = unsigned long long;

constexpr int WIDTH10 = 4;
constexpr int BASE10 = 1e4;

constexpr int WIDTH2 = 8; // 16疑似会掉double精度
constexpr int BASE2 = 1 << 8;

class Big {
public:
  Big();
  Big(int x);
  Big(int modB, int modP);
  Big(int x, int modB, int modP);
  Big(const char number[], int modB = 10, int modP = 0);
  Big(const Big &other);

  Big &operator=(const Big &other);
  Big &operator=(const int &x);

  Big operator+(const Big &b) const;
  Big &operator+=(const Big &b);
  Big operator-(const Big &b) const;
  Big &operator-=(const Big &b);
  Big operator*(const Big &b) const;
  Big &operator*=(const Big &b);
  Big operator/(const Big &b) const;
  Big &operator/=(const Big &b);

  void div2(int base = 10);
  int mod2() const;
  bool iszero() const;
  Big operator^(Big b) const;

  bool operator<(const Big &b) const;
  bool operator==(const Big &b) const;
  bool operator!=(const Big &b) const;
  bool operator>=(const Big &b) const;
  bool operator<=(const Big &b) const;
  bool operator>(const Big &b) const;

  friend class IO;

  constexpr int base() const;
  constexpr int width() const;
  void checkMOD() const;
  void dec2bin();
  void bin2Dec();

private:
  list<i64> digits;
  int modB, modP;

  void fft_recursive(complex *f, int len, int opt) const;
  void fft(complex *f, int n, int opt) const;
  void shift(int shift);
  void trim();
  void mod();
  void carry();
};
#endif