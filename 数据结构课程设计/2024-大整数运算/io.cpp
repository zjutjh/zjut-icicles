
#define DEBUG 1 // 调试开关

// 快速读写，从文件读入时可以注释掉上面的debug，但是交互的时候必须不注释，否则不会显示，
// 在数据量达到1e8级别时提升明显
// 在1e6范围内可以提升10%左右

#include "io.h"
#include "big.h"
#include "big.cpp"

#include <cstdio>

#if DEBUG
IO::IO() {};
IO::~IO() {};
#else
IO::IO() : p1(buf), p2(buf), pp(pbuf) {}

IO::~IO() { fwrite(pbuf, 1, pp - pbuf, stdout); }
#endif
char IO::gc() {
#if DEBUG // 调试，可显示字符
  return getchar();
#endif
  if (p1 == p2)
    p2 = (p1 = buf) + fread(buf, 1, MAXSIZE, stdin);
  return p1 == p2 ? ' ' : *p1++;
}

bool IO::blank(char ch) {
  return ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t' || ch == -1;
}

constexpr bool IO::isdigit(char ch) const { return ch >= '0' && ch <= '9'; }

void IO::read(int &x) {
  double tmp = 1;
  bool sign = false;
  x = 0;
  char ch = gc();
  for (; !isdigit(ch); ch = gc())
    if (ch == '-') {
      write("Negative numbers are not allowed!\n");
      exit(-1);
    }
  for (; isdigit(ch); ch = gc())
    x = x * 10 + (ch - '0');
  if (ch == '.') {
    write("Floating point numbers are not allowed!\n");
    exit(-1);
  }
}

void IO::read(char *s) {
  char ch = gc();
  for (; blank(ch); ch = gc())
    ;
  for (; !blank(ch); ch = gc())
    *s++ = ch;
  *s = 0;
}

void IO::read(char &c) {
  for (c = gc(); blank(c); c = gc())
    ;
}

void IO::read(Big &b) {
  char s[MAXSIZE];
  read(s);
  b = Big(s, b.modB, b.modP);
}

void IO::push(const char &c) {
#if DEBUG // 调试，可显示字符
  putchar(c);
#else
  if (pp - pbuf == MAXSIZE)
    fwrite(pbuf, 1, MAXSIZE, stdout), pp = pbuf;
  *pp++ = c;
#endif
}

void IO::write(int x, int width, int modB) {
  static int sta[90];
  int top = 0;
  do {
    sta[top++] = x % modB, x /= modB;
    if (width != -1)
      width--;
  } while (x || width != -1 && width);
  while (top)
    push(sta[--top] + '0');
}

void IO::write(const char s[]) {
  int i = 0;
  while (s[i]) {
    push(s[i++]);
  }
}

void IO::write(const Big &b) {
  auto i = b.digits.rbegin();
  write(*i, -1, b.modB);
  if (i == b.digits.begin())
    return;
  i--;
  while (1) {
    write(*i, !b.width() ? -1 : b.width(), b.modB);
    if (i == b.digits.begin())
      break;
    i--;
  }
}