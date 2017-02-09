SRCDIR:=$(CURDIR)/../..
include $(SRCDIR)/Makefile.config

LFLAGS=-I. -Ilwip/src/include -Ilwip/src/include/ipv4 #	-Ilwip/src/include/ipv6
CFLAGS+=$(LFLAGS) -Wno-address -Wno-char-subscripts # -Wno-sign-compare
CXXFLAGS+=$(LFLAGS)

COREFILES=$(wildcard lwip/src/core/*.c)
IP4FILES=$(wildcard lwip/src/core/ipv4/*.c)
APIFILES=$(wildcard lwip/src/api/*.c)
NETIFFILES=$(wildcard lwip/src/netif/*.c)

SOURCES=$(COREFILES) $(IP4FILES) $(NETIFFILES) $(APIFILES)
BSOURCES=$(notdir $(SOURCES))
OBJECTS=$(BSOURCES:%.c=%.o) 
CXXSOURCES=$(wildcard *.cc)
BCXXSOURCES=$(notdir $(CXXSOURCES))
CXXOBJECTS=$(BCXXSOURCES:%.cc=%.o)

vpath %.c lwip/src/core lwip/src/core/ipv4 lwip/src/api lwip/src/netif

liblwip.o: $(OBJECTS) $(CXXOBJECTS)
	$(LD) -r -o $@ $^

$(OBJECTS): %.o: %.c $(SRCDIR)/Makefile.config
	$(CC) $(CFLAGS) -c $< -o $@

$(CXXOBJECTS): %.o: %.cc $(SRCDIR)/Makefile.config
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f liblwip.o
	rm -f $(OBJECTS)
	rm -f $(CXXOBJECTS)

vclean: clean
	rm -f Makefile.dep

dep depend Makefile.dep:
	$(CC) -MM $(CFLAGS) $(SOURCES) > Makefile.dep
	$(CXX) -MM $(CXXFLAGS) $(CXXSOURCES) >> Makefile.dep

-include Makefile.dep
