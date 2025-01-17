// 简单的复数实现，用于fft

#ifndef COMPLEX_H
#define COMPLEX_H

class complex {
private:
  double x;
  double y;

public:
  complex(double x = 0.0, double y = 0.0);

  complex operator+(const complex &other) const;
  complex operator-(const complex &other) const;
  complex operator*(const complex &other) const;

  constexpr double real();
  constexpr double imag();
};
#endif