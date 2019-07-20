#include <stdio.h>
#include <stdlib.h>

int quickPower(int num, int power) {
  int rs = 1;
  int base = num;
  while (power != 0) {
    if ((power & 1) == 1)
      rs *= base;
    base *= base;
    power >>= 1;
  }
  return rs;
}

int C(int n, int m) {
  int isum = 1;
  for (int k = 1; k <= n; k++)
    isum =
        (isum * (m - n + k)) / k; //先算乘法，避免先算（m-n+k）/k除不尽带来误差
  return isum;
}

int left(int x, int a, int n) { return quickPower(x + a, n); }

int right(int x, int a, int n) {
  int isum = 0;
  for (int k = 0; k <= n; k++) {
    isum += C(k, n) * quickPower(x, k) * quickPower(a, n - k);
  }
  return isum;
}

int main(int argc, char *argv[]) {
  int x = atoi(argv[1]);
  int a = atoi(argv[2]);
  int n = atoi(argv[3]);
  printf("left:%d\n", left(x, a, n));
  printf("right:%d\n", right(x, a, n));
  return 0;
}
