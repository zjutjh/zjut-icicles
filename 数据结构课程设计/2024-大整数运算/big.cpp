#include "big.h"
#include "complex.h"
#include "io.h"

#include <cmath>

IO io;

const double pi = acos(-1);

complex f[MAXSIZE], g[MAXSIZE], sav[MAXSIZE];

int tr[MAXSIZE], buf[MAXSIZE];

Big::Big() { digits.push_back(0); }
Big::Big(int x) {
  digits.clear();
  if (!x) { // 避免链表为空导致的一系列内存问题
    digits.push_back(0);
  }
  while (x) {
    digits.push_back(x % base());
    x /= base();
  }
}
Big::Big(int modB, int modP) : modB(modB), modP(modP) {
  checkMOD();
}
Big::Big(int x, int modB, int modP) {
  this->modB = modB;
  this->modP = modP;
  
  checkMOD();

  digits.clear();
  if (!x) {
    digits.push_back(0);
  }
  while (x) {
    digits.push_back(x % base());
    x /= base();
  }
}

// 从字符串读入，不使用std::string以提升性能
Big::Big(const char number[], int modB, int modP) {
  this->modB = modB;
  this->modP = modP;
  
  checkMOD();

  int tmp = 0, len = 0;
  while (number[len]) {
    len++;
  }
  for (int r = len; r > 0; r -= WIDTH10) { // 十进制转压位十进制
    int l = std::max(r - WIDTH10, 0);
    tmp = 0;
    for (int i = l; i < r; i++) {
      tmp = tmp * 10 + number[i] - '0';
    }
    digits.push_back(tmp);
  }
  if (modB == 2) {
    dec2bin();
  }
  mod();
}

Big::Big(const Big &other) {
  digits = other.digits;
  modB = other.modB;
  modP = other.modP;
}

Big &Big::operator=(const Big &other) {
  digits = other.digits;
  modB = other.modB;
  modP = other.modP;
  return *this;
}

Big &Big::operator=(const int &x) {
  *this = Big(x, 10, 0);
  return *this;
}

// 简单的依次相加即可
// O(n)
Big Big::operator+(const Big &b) const {
  Big c(modB, modP);
  auto i1 = digits.begin(), i2 = b.digits.begin();
  while (i1 != digits.end() && i2 != b.digits.end()) {
    c.digits.push_back(*i1 + *i2);
    i1++, i2++;
  }
  while (i1 != digits.end()) {
    c.digits.push_back(*i1);
    i1++;
  }
  while (i2 != b.digits.end()) {
    c.digits.push_back(*i2);
    i2++;
  }
  c.digits.push_back(0);
  c.carry();
  c.trim();
  c.mod();
  return c;
};
Big &Big::operator+=(const Big &b) {
  *this = *this + b;
  return *this;
}

// 简单的借位即可
// O(n)
Big Big::operator-(const Big &b) const {
  Big a = *this;
  if (a < b) {
    io.write("Error: negative number\n");
    exit(-1);
  }
  auto i1 = a.digits.begin();
  auto i2 = b.digits.begin();
  while (i2 != b.digits.end()) {
    if (*i1 < *i2) {
      auto j = i1;
      j++;
      while (!*j)
        j++;
      while (j != i1) {
        --*j;
        j--;
        *j += base();
      }
    }
    *i1 -= *i2;
    i1++, i2++;
  }
  a.trim();
  a.mod();
  return a;
};
Big &Big::operator-=(const Big &b) {
  *this = *this - b;
  return *this;
}
// 乘法使用了循环实现的fft，用了位逆序置换和蝶形运算优化，可以降低常数，
// 复杂度O(nlogn)
Big Big::operator*(const Big &b) const {
  int n = digits.size(), m = b.digits.size();
  m += n, n = 1;
  while (n < m) // 长度必须为2的幂，因此在前面补零
    n <<= 1;
  // 我也不确定要不要这么做，但是初始化可以避免奇怪的问题
  for (int i = 0; i < n; i++) {
    f[i] = g[i] = 0;
  }
  int i = 0;
  for (auto it = digits.begin(); it != digits.end(); it++) {
    f[i++] = {double(*it), 0};
  }
  i = 0;
  for (auto it = b.digits.begin(); it != b.digits.end(); it++) {
    g[i++] = {double(*it), 0};
  }
  for (int i = 0; i < n; i++) {
    tr[i] = (tr[i >> 1] >> 1) | (i & 1 ? n >> 1 : 0);
    // 位逆序置换，数位dp的思想，提前放好位置
  }
  fft(f, n, 1);
  fft(g, n, 1);
  // 将多项式的系数表达转换为点值表达，从而降低运算复杂度
  for (int i = 0; i < n; i++) {
    f[i] = f[i] * g[i];
  }
  // 将多项式的点值表达转换为系数表达
  fft(f, n, -1);
  i = 0;
  Big c(modB, modP);
  while (i < m) {
    c.digits.push_back(f[i++].real() / n + 0.5);
  }
  c.carry(); // 同样需要处理进位
  c.trim();
  c.mod();
  return c;
};

Big &Big::operator*=(const Big &b) {
  *this = *this * b;
  return *this;
};
// 采用倍增，因为更快的O(nlogn)的算法实在来不及学了
// 复杂度 O(n^2)
Big Big::operator/(const Big &b) const {
  if (*this < b) {
    return Big(0, 10, 0);
  }
  Big x = b, y(1, modB, modP);
  // std::list<Big> s1, s2;
  list<Big> s1, s2;

  for (; x <= *this; x += x, y += y)
    s1.push_back(x), s2.push_back(y);

  x = *this;
  Big ans(0, 10, 0);

  for (; !s1.empty(); s1.pop_back(), s2.pop_back())
    if (s1.back() <= x) {
      x -= s1.back();
      ans += s2.back();
    }
  return ans;
};
Big &Big::operator/=(const Big &b) {
  *this = *this / b;
  return *this;
};
// 方便快速幂
void Big::div2(int base) {
  int carry = 0;
  for (auto it = digits.rbegin();; --it) {
    int new_digit = *it + carry * base;
    *it = new_digit / 2;
    carry = new_digit % 2;
    if (it == digits.begin())
      break;
  }
  trim();
}
// 同样是方便快速幂
int Big::mod2() const { return digits.front() % 2; }
bool Big::iszero() const {
  // return digits.size() == 1 && digits.front() == 0 || digits.empty();
  for (auto &i : digits) {
    if (i) {
      return false;
    }
  }
  return true;
}
// 朴素的快速幂实现
// 复杂度约为O(nmlognm)，但实际上常数很小，因此跑得还算快
// n，m为底数和指数的位数，考虑最后一步的复杂度是两个nm位的数相乘，因此远远跑不满
Big Big::operator^(Big b) const {
  Big ans(1, modB, modP), a(*this);
  ans.mod();
  a.mod();
  while (!b.iszero()) {
    if (b.mod2())
      ans = ans * a, ans.mod();
    a = a * a, a.mod();
    b.div2(base());
  }
  ans.trim();
  ans.mod();
  return ans;
};


// 我们只需要一个 <
// 就可以得到其他所有偏序关系，
// 但是由于需要取模，因此大小比较基本上没有意义，仅仅用于避免出现负数
bool Big::operator<(const Big &b) const {
  if (digits.size() != b.digits.size()) {
    return digits.size() < b.digits.size();
  }
  for (auto i1 = digits.rbegin(), i2 = b.digits.rbegin();;) {
    if (*i1 != *i2) {
      return *i1 < *i2;
    }
    if (i1 == digits.begin())
      break;
    i1--, i2--;
  }
  return false;
}
bool Big::operator==(const Big &b) const {return !(*this < b) && !(b < *this);}
bool Big::operator!=(const Big &b) const { return !(*this == b); }
bool Big::operator>=(const Big &b) const { return !(*this < b); }
bool Big::operator<=(const Big &b) const { return *this < b || *this == b; }
bool Big::operator>(const Big &b) const { return !(*this <= b); }

// 递归实现的fft，方便理解的初级版本，主要利用了单位根的性质和分治思想
// 从而加速DFT，复杂度O(nlogn)
void Big::fft_recursive(complex *f, int len, int opt) const {
  if (len == 1)
    return;
  int mid = len / 2;
  for (int i = 0; i < len; i++) {
    sav[i] = f[i];
  }
  complex *fl = f, *fr = f + mid;
  for (int i = 0; i < mid; i++) {
    fl[i] = sav[i << 1], fr[i] = sav[i << 1 | 1];
  }
  fft_recursive(fl, mid, opt);
  fft_recursive(fr, mid, opt);
  complex w(cos(2 * pi / len), sin(2 * pi / len) * opt);
  complex c(1, 0);
  for (int i = 0; i < mid; i++) {
    sav[i] = fl[i] + c * fr[i];
    sav[i + mid] = fl[i] - c * fr[i];
    c = c * w;
  }
  for (int i = 0; i < len; i++) {
    f[i] = sav[i];
  }
}

// 常数更小的fft的循环写法，洛谷P1919比上面的版本快一倍，虽然时间复杂度相同
void Big::fft(complex *f, int n, int opt) const {
  if (n == 1)
    return;
  for (int i = 0; i < n; i++) {
    if (i < tr[i]) {
      std::swap(f[i], f[tr[i]]);
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    complex wn(cos(2 * pi / len), sin(2 * pi / len) * opt);
    int mid = len >> 1;
    for (int l = 0; l < n; l += len) {
      int r = l + mid;
      complex w(1, 0), t;
      for (int i = l; i < r; i++) {
        // 因为double的精度损失，为了压位，我们需要重新计算准确的wn值
        // 如果不弄，最多压2位，也就是100，现在我们压4位
        if (!((i - l) % 1024))
          w = complex(cos(2 * pi * (i - l) / len),
                      sin(2 * pi * (i - l) / len) * opt);
        t = f[i + mid] * w;
        f[i + mid] = f[i] - t;
        f[i] = f[i] + t;
        w = w * wn; // 这里会导致double的精度损失
      }
    }
  }
}

// 用于快速除法的工具函数，但是来不及学了
void Big::shift(int shift) {
  if (shift >= 0) {
    for (int i = 0; i < shift; i++) {
      digits.push_front(0);
    }
  } else {
    shift = -shift;
    for (int i = 0; i < shift; i++) {
      digits.pop_front();
    }
  }
}

// 去除前导零
void Big::trim() {
  while (digits.size() > 1 && digits.back() == 0) {
    digits.pop_back();
  }
}

// 根据底数压位后就可以很方便的取模
void Big::mod() {
  while (modP && digits.size() * width() > modP) {
    digits.pop_back();
  }
}

// 处理借位
void Big::carry() {
  auto i = digits.begin(), j = ++digits.begin();
  while (j != digits.end()) {
    *j += *i / base();
    *i %= base();
    i++, j++;
  }
}

// 动态确定当前的BASE和WIDTH
constexpr int Big::base() const {return modB == 2 ? BASE2 : BASE10;}
constexpr int Big::width() const {return modB == 2 ? WIDTH2 : WIDTH10;}

// 检查模数是否合法，当前仅支持模底数为2或10，
// 当模底数为2时，模指数必须为8的倍数，当模底数为10时，模指数必须为4的倍数
void Big::checkMOD() const {
  if (modP % width() || modP == 1) {
    io.write(modP);
    io.write("\nError: modP must be 0 or a multiple of width\n");
    exit(-1);
  }
  if (modB != 10 && modB != 2) {
    io.write(modB);
    io.write("\nError: modB must be 10 or 2\n");
    exit(-1);
  }
}

// 压位十进制转压位二进制，因为我们只考虑了十进制输入，因此不能直接从二进制读入，需要从十进制转换
void Big::dec2bin() { // 改变内部压位规则，不取模
  int len = 0;
  while (!iszero()) {
    buf[len++] = mod2();
    div2(BASE10);
  }
  list<i64> digits2;
  for (int l = 0; l < len; l += WIDTH2) { // 由于前面是倒着的，因此这里从最低位到最高位
    int r = std::min(l + WIDTH2, len);
    int tmp = 0;
    for (int i = r - 1; i >= l; i--) {
      tmp = tmp * 2 + buf[i];
    }
    digits2.push_back(tmp);
  }
  digits = digits2;
}
