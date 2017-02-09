#include "generic/Bitmap.h"

static const size_t bs = 64*8;

int main() {
  Bitmap<bs> bm;
  cout << (bm.empty() ? "empty" : "not empty") << endl;
  mword x;
  while (cin >> x) {
    if (x < bs) bm.set(x);
    else break;
  }
  while (cin >> x) {
    if (x < bs) cout << (bm.test(x) ? "yes" : "no") << endl;
    else break;
  }
  cout << (bm.empty() ? "empty" : "not empty") << endl;
  cout << bm.count() << endl;
  cout << bm.findset() << endl;
  cout << bm.findset_rev() << endl;
  cout << bm.findclear() << endl;
}
