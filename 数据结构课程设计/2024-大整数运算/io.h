#ifndef IO_H
#define IO_H

#define MAXSIZE 1 << 22

#include "big.h"

class IO {
  char buf[MAXSIZE], *p1, *p2;
  char pbuf[MAXSIZE], *pp;

public:
  IO();
  ~IO();

  char gc();
  bool blank(char ch);
  constexpr bool isdigit(char ch) const;
  void read(int &x);
  void read(char *s);
  void read(char &c);
  void read(Big &b);
  void push(const char &c);
  void write(int x, int width = -1, int base = 10);
  void write(const char s[]);
  void write(const Big &b);
};

#endif