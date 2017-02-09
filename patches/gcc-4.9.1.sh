echo '#define LINK_LIBGCC_SPEC "%D -L' $2'"' >> $1/gcc/config/i386/i386elf.h
echo '#define LIBGCC_SPEC "-lgcc -lKOS -lc"'>> $1/gcc/config/i386/i386elf.h
