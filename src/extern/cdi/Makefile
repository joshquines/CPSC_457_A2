SRCDIR:=$(CURDIR)/../..
include $(SRCDIR)/Makefile.config

LFLAGS=-Iinclude
CFLAGS+=-include cdi_glue.h $(LFLAGS) -Wno-format -Wno-sign-compare
CXXFLAGS+=$(LFLAGS)

SOURCES=$(wildcard */*.c)
BSOURCES=$(notdir $(SOURCES))
OBJECTS=$(BSOURCES:%.c=%.o)
CXXSOURCES=$(wildcard *.cc)
BCXXSOURCES=$(notdir $(CXXSOURCES))
CXXOBJECTS=$(BCXXSOURCES:%.cc=%.o)

vpath %.c e1000

libcdi.o: $(OBJECTS) $(CXXOBJECTS)
	$(LD) -r -o $@ $^

$(OBJECTS): %.o: %.c $(SRCDIR)/Makefile.config
	$(CC) $(CFLAGS) -c $< -o $@

$(CXXOBJECTS): %.o: %.cc $(SRCDIR)/Makefile.config
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f libcdi.o
	rm -f $(OBJECTS)
	rm -f $(CXXOBJECTS)

vclean: clean
	rm -f Makefile.dep

dep depend Makefile.dep:
	$(CC) -MM $(CFLAGS) $(SOURCES) > Makefile.dep
	$(CXX) -MM $(CXXFLAGS) $(CXXSOURCES) >> Makefile.dep

-include Makefile.dep
