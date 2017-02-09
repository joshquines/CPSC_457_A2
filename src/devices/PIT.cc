#include "machine/APIC.h"
#include "machine/CPU.h"
#include "machine/Machine.h"
#include "devices/PIT.h"

// http://www.jamesmolloy.co.uk/tutorial_html/5.-IRQs%20and%20the%20PIT.html
void PIT::init() {
	Machine::registerIrqSync(PIC::PIT, 0xf0);
	uint32_t divisor = 1193182 / frequency; // base frequency is 1193181.666 Hz
	CPU::out8(0x43, 0x36);           // command: binary counting, mode 3, channel 0
	CPU::out8(0x40, divisor & 0xFF); // frequency divisor LSB
	CPU::out8(0x40, divisor >> 8);   // frequency divisor MSB
}
