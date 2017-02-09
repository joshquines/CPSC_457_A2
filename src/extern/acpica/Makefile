#
# Common make for acpica tools and utilities
#

#
# Note: This makefile is intended to be used from within the native
# ACPICA directory structure, from under top level acpica directory.
# It specifically places all the object files for each tool in separate
# generate/unix subdirectories, not within the various ACPICA source
# code directories. This prevents collisions between different
# compilations of the same source file with different compile options.
#

#ifeq ($(OS),efi)
#	BUILD_DIRECTORY_PATH = "generate/efi"
#	include generate/efi/Makefile.config
#	include generate/efi/Makefile.common
#else
#	BUILD_DIRECTORY_PATH = "generate/unix"
#	include generate/unix/Makefile.config
#	include generate/unix/Makefile.common
#endif
SRCDIR:=$(CURDIR)/../..
include $(SRCDIR)/Makefile.config
CC=$(GCC) # compiling with clang is possible, but throws warnings
CFLAGS += -I$(ACPICA_INCLUDE) -D__KOS__ -Wstrict-aliasing=0 $(OPTFLAGS)

ACPICA_SRC =            source
ACPICA_COMMON =         $(ACPICA_SRC)/common
ACPICA_TOOLS =          $(ACPICA_SRC)/tools
ACPICA_OSL =            $(ACPICA_SRC)/os_specific/service_layers
ACPICA_CORE =           $(ACPICA_SRC)/components
ACPICA_INCLUDE =        $(ACPICA_SRC)/include
ACPICA_DEBUGGER =       $(ACPICA_CORE)/debugger
ACPICA_DISASSEMBLER =   $(ACPICA_CORE)/disassembler
ACPICA_DISPATCHER =     $(ACPICA_CORE)/dispatcher
ACPICA_EVENTS =         $(ACPICA_CORE)/events
ACPICA_EXECUTER =       $(ACPICA_CORE)/executer
ACPICA_HARDWARE =       $(ACPICA_CORE)/hardware
ACPICA_NAMESPACE =      $(ACPICA_CORE)/namespace
ACPICA_PARSER =         $(ACPICA_CORE)/parser
ACPICA_RESOURCES =      $(ACPICA_CORE)/resources
ACPICA_TABLES =         $(ACPICA_CORE)/tables
ACPICA_UTILITIES =      $(ACPICA_CORE)/utilities

ACPICA=$(ACPICA_DISPATCHER)\
	$(ACPICA_EVENTS)\
	$(ACPICA_EXECUTER)\
	$(ACPICA_HARDWARE)\
	$(ACPICA_NAMESPACE)\
	$(ACPICA_PARSER)\
	$(ACPICA_RESOURCES)\
	$(ACPICA_TABLES)\
	$(ACPICA_UTILITIES)

SOURCES=$(wildcard $(addsuffix /*.c,$(ACPICA)))

OBJECTS=$(subst .c,.o,$(notdir $(SOURCES)))

vpath %.c $(ACPICA)

libacpica.a: $(SOURCES)
	$(MAKE) $(OBJECTS)
	$(AR) r $@ $(OBJECTS)
	$(RANLIB) $@
	rm -rf $(OBJECTS)

clean:
	rm -rf $(OBJECTS)

vclean: clean
	rm -rf libacpica.a

dep depend Makefile.dep:
