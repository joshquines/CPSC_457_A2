SRCDIR:=$(CURDIR)/..
include $(SRCDIR)/Makefile.config

CXXFLAGS=$(CFGFLAGS) $(OPTFLAGS) $(DBGFLAGS) $(LANGFLAGS) $(MACHFLAGS)
ifeq ($(CC),clang)
CXXFLAGS+=$(CLANGFLAGS)
endif

SRC=$(wildcard *.cc)
OBJ=$(SRC:%.cc=%.o)
ASRC=$(wildcard *.S)
AOBJ=$(ASRC:%.S=%.o)
LIB=libKOS.a

all: $(LIB)

$(LIB): $(OBJ) $(AOBJ) malloc.o
	$(AR) cr $@ $^

malloc.o: $(CURDIR)/../extern/dlmalloc/malloc.c malloc_glue.h
	$(CXX) $(CXXFLAGS) -c $< -o $@ -include malloc_glue.h -DDEBUG

$(OBJ): %.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(AOBJ): %.o: %.S
	$(AS) $(ASFLAGS) $< -o $@

clean:
	rm -f $(LIB) $(OBJ) $(AOBJ) malloc.o

vclean: clean
	rm -f Makefile.dep

dep depend Makefile.dep:
	$(CXX) -MM $(CXXFLAGS) $(SRC) > Makefile.dep

-include Makefile.dep
