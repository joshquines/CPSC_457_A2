#include "generic/ManagedArray.h"

void kassertprints(char const*, int, char const*) {}
void kassertprinte(unsigned long long) {}
void Reboot(unsigned long) { unreachable(); }

int main() {
  ManagedArray<long long, allocator> a(4);
  int x;
  while (cin >> x) {
    if (x >= 0) a.put(x);
    else break;
  }
  while (cin >> x) {
    if (x >= 0) a.remove(x);
    else break;
  }
  while (cin >> x) {
    if (x >= 0) a.set(a.reserveIndex(), x);
    else break;
  }
  for (int i = 0; i < a.currentIndex(); i += 1) {
    if (a.valid(i)) cout << ' ' << a.get(i);
  }
  cout << ' ' << a.size() << ' ' << a.currentCapacity() << endl;
}
