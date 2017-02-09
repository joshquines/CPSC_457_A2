#include "generic/Bitmap.h"

static const size_t bc = 1ull << 20;

int main() {
  HierarchicalBitmap<512,40> hbm;
  char* p = new char[hbm.allocsize(bc)];
  hbm.init(bc, p);
  cout << (hbm.empty() ? "empty" : "not empty") << endl;
  mword x;
  while (cin >> x) {
    if (x < bc) hbm.set(x);
    else break;
  }
  while (cin >> x) {
    if (x < bc) hbm.clear(x);
    else break;
  }
  while (cin >> x) {
    if (x < bc) {
      cout << (hbm.test(x) ? "yes" : "no") << endl;
    } else break;
  }
  cout << (hbm.empty() ? "empty" : "not empty") << endl;
  cout << hbm.findset() << endl;
  cout << hbm.findset_rev() << endl;
  return 0;
}
