#include "runtime/Thread.h"
#include "kernel/Output.h"
#include "machine/APIC.h"
#include "machine/Machine.h"

extern void (*tipiHandler)(void);

// simple IPI measurements, TODO: should do IPI ping pong
namespace IPI_Experiment {

static volatile bool done = false;

static volatile mword tipiCount = 0;
static void tipiCounter() { tipiCount += 1; }

static mword rCount = 0;
static void receiver() { while (!done) rCount += 1; }

static mword sCount = 0;
static mword tscStart, tscEnd;
static void sender() {
  tscStart = CPU::readTSC();
  for (int i = 0; i < 1000; i += 1) {
    mword tc = tipiCount;
    Machine::sendIPI(1, APIC::TestIPI);
    while (tc == tipiCount) sCount += 1;
  }
  tscEnd = CPU::readTSC();
  done = true;
}

static void run() {
  KOUT::outl("IPI experiment running...");
  tipiHandler = tipiCounter;
  Thread* t = Thread::create();
  Machine::setAffinity(*t, 1);
  t->start((ptr_t)receiver);
  t = Thread::create();
  Machine::setAffinity(*t, 2);
  t->start((ptr_t)sender);
  while (!done);
  KOUT::outl("IPI experiment results: ", tipiCount, ' ', sCount, ' ', rCount, ' ', tscEnd - tscStart);
}

} // namespace IPI_Experiment

int Experiments() {
  IPI_Experiment::run();
  return 0;
}
