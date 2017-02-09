SRCDIR:=$(CURDIR)/..
include $(SRCDIR)/Makefile.config

UserMain.h: $(SRCDIR)/Makefile.config
	rm -f $@
	./setup_usermain.sh $(MODULES)

clean:

vclean:
	rm -f UserMain.h

dep depend Makefile.dep:
