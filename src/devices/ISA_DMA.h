#ifndef _ISA_DMA_h_
#define _ISA_DMA_h_ 1

#include "generic/bitmanip.h"

// ISA DMA  http://wiki.osdev.org/DMA

class ISA_DMA {
  ISA_DMA() = delete;
  ISA_DMA(const ISA_DMA&) = delete;
  ISA_DMA& operator=(const ISA_DMA&) = delete;

//  static const BitString<uint8_t, 0, 2> Selector;
  static const BitString<uint8_t, 2, 1> MaskOn;

  // Mode
  static const BitString<uint8_t, 0, 2> Selector;
  static const BitString<uint8_t, 2, 2> Transfer;
  static const BitString<uint8_t, 4, 1> Auto;
  static const BitString<uint8_t, 5, 1> Down;
  static const BitString<uint8_t, 6, 2> Mode;

  enum class TransferType {
    SelfTest = 0b00,
    Write    = 0b01,
    Read     = 0b10,
    Invalid  = 0b11
  };

  enum class TransferMode {
    OnDemand = 0b00,
    Single   = 0b01,
    Block    = 0b10,
    Cascade  = 0b11
  };

  static inline void mask(uint8_t channel, bool mask);
  static inline void setup(uint8_t channel, uint8_t mode, bool autoInit, paddr buffer, uint16_t length);
public:
  static bool startTransfer(uint8_t channel, uint8_t mode, paddr buffer, size_t length);
};

#endif /* _ISA_DMA_h_ */
