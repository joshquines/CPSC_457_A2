help:
	@echo "USAGE: (see src/Makefile for additional targets)"
	@echo "$(MAKE) all      build everything"
	@echo "$(MAKE) clean    clean everything"
	@echo "$(MAKE) dep      build dependencies"
	@echo "$(MAKE) run      build and run (qemu)"
	@echo "$(MAKE) debug    build, run (qemu), and debug (qemu/gdb)"
	@echo "$(MAKE) gdb      build, run (qemu), and debug (remote gdb)"
	@echo "$(MAKE) bochs    build and run/debug (bochs)"
	@echo "$(MAKE) vbox     build and run/debug (VirtualBox) - setup needed"

tgz: vclean
	rm -f kos.tgz; tar czvf kos.tgz --xform 's,,kos/,' \
	--exclude src/extern/acpica --exclude src/extern/lwip/lwip \
	--exclude cfg/Logs --exclude cfg/KOS.vbox --exclude cfg/KOS.vbox-prev \
	cfg config LICENSE Makefile patches README setup_crossenv.sh src

.DEFAULT:
	nice -10 $(MAKE) -C src -j $(shell fgrep processor /proc/cpuinfo|wc -l) $@
