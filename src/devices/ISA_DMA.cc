#include "machine/CPU.h"
#include "devices/ISA_DMA.h"

// SingleChannelMask
//static const BitString<uint8_t, 0, 2> ISA_DMA::Selector;
const BitString<uint8_t, 2, 1> ISA_DMA::MaskOn;

// Mode
const BitString<uint8_t, 0, 2> ISA_DMA::Selector;
const BitString<uint8_t, 2, 2> ISA_DMA::Transfer;
const BitString<uint8_t, 4, 1> ISA_DMA::Auto;
const BitString<uint8_t, 5, 1> ISA_DMA::Down;
const BitString<uint8_t, 6, 2> ISA_DMA::Mode;

static struct {
  uint8_t Address;
  uint8_t Count;
  uint8_t Page;
} ports[] = {
  { 0x00,0x01,0x87 },
  { 0x02,0x03,0x83 },
  { 0x04,0x05,0x81 },
  { 0x06,0x07,0x82 },
  { 0xc0,0xc2,0x8f },
  { 0xc4,0xc6,0x8b },
  { 0xc8,0xca,0x89 },
  { 0xcc,0xce,0x8a }
};

static struct {
  uint8_t Status;
  uint8_t Command;
  uint8_t Request;
  uint8_t SingleChannelMask;
  uint8_t Mode;
  uint8_t FFReset;
  uint8_t Intermediate;
  uint8_t MasterReset;
  uint8_t MaskReset;
  uint8_t MultiChannelMask;
} registers[] = {
  { 0x08, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0d, 0x0e, 0x0f },
  { 0xd0, 0xd0, 0xd2, 0xd4, 0xd6, 0xd8, 0xda, 0xda, 0xdc, 0xde }
};

inline void ISA_DMA::mask(uint8_t channel, bool mask) {
  CPU::out8(registers[channel/4].SingleChannelMask,
    (channel & Selector()) | MaskOn.put(mask));
}

inline void ISA_DMA::setup(uint8_t channel, uint8_t mode, bool autoInit, paddr buffer, uint16_t length) {
  CPU::out8(registers[channel/4].FFReset, 0xff);           // reset flip-flop
  CPU::out8(registers[channel/4].Mode,
    (channel & Selector()) | (mode & (Transfer() | Auto.put(autoInit) | Mode())));
  CPU::out8(ports[channel].Address,  buffer        & 0xff); // 8-bit transfer
  CPU::out8(ports[channel].Address, (buffer >>  8) & 0xff);
  CPU::out8(ports[channel].Page,    (buffer >> 16) & 0xff);
  CPU::out8(ports[channel].Count,    length        & 0xff);
  CPU::out8(ports[channel].Count,   (length >>  8) & 0xff);
}

bool ISA_DMA::startTransfer(uint8_t channel, uint8_t mode, paddr buffer, size_t length) {
  if (channel >= 8 || channel/4 == 0) return false;
  if (length > 0x10000) return false; // transfer limited to 64kb
  mask(channel, true);                // mask DRQ until setup done
  setup(channel, mode, false, length-1, buffer);
  mask(channel, false);               // unmask DRQ
  return true;
}
