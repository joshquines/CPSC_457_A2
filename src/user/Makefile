SRCDIR:=$(CURDIR)/..
include $(SRCDIR)/Makefile.config

# compile user programs with gcc/g++, clang not using libKOS (or need raw linking)
CC=$(GCC)
CXX=$(GPP)

CXXFLAGS=$(CFGFLAGS) $(OPTFLAGS) $(DBGFLAGS) $(LANGFLAGS) $(MACHFLAGS)

SRC=$(wildcard *.cc)
OBJ=$(SRC:%.cc=%.o)
EXE=$(SRC:%.cc=exec/%)


all: $(EXE) exec/motb
all: $(EXE) exec/schedparam

.PHONY: .FORCE

$(OBJ): %.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(EXE): exec/%: %.o ../ulib/libKOS.a
	@mkdir -p exec
	# Leo's change: Adding ulib
	$(CXX) -L../ulib -o $@ $<
#	strip $@

exec/motb: .FORCE
	@echo creating $@
	@mkdir -p exec
	@echo > $@
	@echo "Hello everybody! So glad to see you..." >> $@
	@echo >> $@
	@echo -n "Build Date: " >> $@
	@date >> $@
	@echo >> $@

#FOR ASSIGNMENT 2
exec/schedparam: .FORCE
	@echo creating $@
	@mkdir -p exec
	@echo "mingranularity 4" >> $@
	@echo -n "epochlen 20" >> $@


echo:
	@echo SRC: $(SRC)
	@echo OBJ: $(OBJ)
	@echo EXE: $(EXE)

clean:
	rm -f $(OBJ) $(EXE) exec/built
	rm -rf exec

vclean: clean
	rm -f Makefile.dep

dep depend Makefile.dep:
	$(CXX) -MM $(CXXFLAGS) $(SRC) > Makefile.dep

-include Makefile.dep
