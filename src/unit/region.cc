#include "generic/RegionSet.h"

int main() {
  RegionSet<Region<mword>> rs;
  char c;
  int s, e;
  while (cin >> c >> s >> e) {
    switch (tolower(c)) {
      case 'i': rs.insert(Region<mword>(s,e)); break;
      case 'r': rs.remove(Region<mword>(s,e)); rs.print<true>(cout); cout << endl; break;
      case 'o': cout << (rs.out(Region<mword>(s,e)) ? " out" : " not out") << endl; break;
      default: cout << "unknown cmd: " << c << endl;
    }
  }
  rs.print<true>(cout); cout << endl;
  return 0;
}
