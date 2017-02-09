ENTRY(entryPoint)

SECTIONS
{
	. = 0x100000;

	/* this must be first, so that grub/multiboot finds it */
	.boot.boot ALIGN(0x1000) : AT(ADDR(.boot.boot)) {
		*(.boot.boot*)
  }

	. += KERNBASE;

	.boot.text ALIGN(0x1000) : AT(ADDR(.boot.text) - KERNBASE) {
		*(.boot.text*)
	}

	.boot.data ALIGN(0x1000) : AT(ADDR(.boot.data) - KERNBASE) {
		*(.boot.data*)
	}

	.ctors ALIGN(0x1000) : AT(ADDR(.ctors) - KERNBASE) {
		*(.ctors*)
	}

	.text ALIGN(0x200000) : AT(ADDR(.text) - KERNBASE) {
		*(.text*)
		*(.gnu.linkonce.text*)
	}

	.rodata ALIGN(0x1000) : AT(ADDR(.rodata) - KERNBASE) {
		*(.rodata*)
		*(.gnu.linkonce.rodata*)
	}

/*
	.gcc_except_table : {
		*(.gcc_except_table*)
	}
*/

	.data ALIGN(0x200000) : AT(ADDR(.data) - KERNBASE) {
		*(.data*)
		*(.gnu.linkonce.data*)
	}

	.bss ALIGN(0x1000) : AT(ADDR(.bss) - KERNBASE) {
		*(COMMON)
		*(.bss*)
		*(.gnu.linkonce.b*)
	}

	/DISCARD/ : {
		*(.comment*)
		*(.eh_frame*)
		*(.gcc_except_table*)
	}

	__MultibootHdr   = ALIGN(0x200000) - KERNBASE;

	__KernelBoot     = ADDR(.boot.boot) + KERNBASE;
	__KernelBootEnd  = ADDR(.boot.data) + SIZEOF(.boot.data);
	__KernelCtors    = ADDR(.ctors);
	__KernelCtorsEnd = ADDR(.ctors) + SIZEOF(.ctors);
	__KernelCode     = ADDR(.text);
	__KernelCodeEnd  = ADDR(.text) + SIZEOF(.text);
	__KernelRO       = ADDR(.rodata);
	__KernelRO_End   = ADDR(.rodata) + SIZEOF(.rodata);
	__KernelData     = ADDR(.data);
	__KernelDataEnd  = ADDR(.data) + SIZEOF(.data);
	__KernelBss      = ADDR(.bss);
	__KernelBssEnd   = ADDR(.bss) + SIZEOF(.bss);

	__loadStart      = __KernelBoot - KERNBASE;
	__loadEnd        = __KernelBss  - KERNBASE;
}
