#ifndef _PIT_h_
#define _PIT_h_ 1

class PIT {
  static const int frequency = 1000;
public:
  void init()                                          __section(".boot.text");
};

#endif /* _PIT_h_ */
