#!/bin/bash
function resolve() {
	echo info symbol $1|gdb $2|fgrep "(gdb)"|head -1
}

if [ $# -lt 1 ]; then
	for i in $(fgrep "XBT:" /tmp/KOS.serial|cut -f2 -d:); do
		resolve $i kernel.sys.debug
	done
elif [ $# -lt 2 ]; then
	resolve $1 kernel.sys.debug
else
	resolve $1 $2
fi
exit 0
