#include "complex.h"

complex::complex(double x, double y) : x(x), y(y) {}

complex complex::operator+(const complex &other) const {
  return complex(x + other.x, y + other.y);
}

complex complex::operator-(const complex &other) const {
  return complex(x - other.x, y - other.y);
}

complex complex::operator*(const complex &other) const {
  return complex(x * other.x - y * other.y, x * other.y + y * other.x);
}

constexpr double complex::real() { return x; }
constexpr double complex::imag() { return x; }
