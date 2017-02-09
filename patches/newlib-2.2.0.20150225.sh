echo "\
.section .text
.global _start
_start:
	# Set up end of the stack frame linked list.
	xor %rbp, %rbp       # clear %rbp
	pushq %rbp           # previous %rip = 0
	pushq %rbp           # previous %rbp = 0
	movq %rsp, %rbp

	# We need those in a moment when we call main.
	pushq %rsi
	pushq %rdi

	# Prepare signals, memory allocation, stdio and such.
 	call _initialize_KOS_standard_library

	# Run the global constructors.
 	call _init

	# Restore argc and argv.
 	popq %rdi
 	popq %rsi

	# Run main
 	call main

	# Terminate the process with the exit code.
 	movq %rax, %rdi
	call exit"\
> $1/libgloss/libnosys/crt0.S

echo "\
.section .init
.global _init
_init:
	 push %rbp
	 movq %rsp, %rbp
	 /* gcc will nicely put the contents of crtbegin.o's .init section here. */

.section .fini
.global _fini
_fini:
	push %rbp
	movq %rsp, %rbp
	/* gcc will nicely put the contents of crtbegin.o's .fini section here. */"\
> $1/libgloss/libnosys/crti.S

echo "\
.section .init
	/* gcc will nicely put the contents of crtend.o's .init section here. */
	popq %rbp
	ret

.section .fini
	/* gcc will nicely put the contents of crtend.o's .fini section here. */
	popq %rbp
	ret"\
> $1/libgloss/libnosys/crtn.S

sed -i 's/OUTPUTS = /OUTPUTS = crt0.o crti.o crtn.o /' $1/libgloss/libnosys/Makefile.in

sed -i 's/#define _READ_WRITE_RETURN_TYPE int/#define _READ_WRITE_RETURN_TYPE _ssize_t/' $1/newlib/libc/include/sys/config.h

echo 'newlib_cflags="${newlib_cflags} -DMALLOC_PROVIDED -DABORT_PROVIDED"'\
	>> $1/newlib/configure.host

cd $1/libgloss
autoconf
