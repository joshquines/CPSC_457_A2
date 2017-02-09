#!/bin/bash
#
# Written by Martin Karsten (mkarsten@uwaterloo.ca)
#
# also see http://wiki.osdev.org/Loopback_Device for other ideas
#
source $(dirname $0)/../../config

setup_usb() {
	[ $(id -u) -eq 0 ] || echo "must run this as root"
    
  MNTDIR=$(mount|fgrep $1|cut -f3 -d' ')
  [ -z "$MNTDIR" ] || {
    umount $MNTDIR || { echo "cannot unmount $MNTDIR"; exit 1; };
  }
      
  printf "o\nn\n\n\n\n\nt\nb\nw\n"|fdisk $1
  mkfs.vfat ${1}1
      
  TMPDIR=$(mktemp -d /tmp/mount.XXXXXXXXXX)
  mount ${1}1 $TMPDIR

	$GRUBDIR/sbin/grub-install --no-floppy --target=i386-pc\
		--root-directory=$TMPDIR $1
	cp -a $2 $TMPDIR
	umount $TMPDIR
	rmdir $TMPDIR
	exit 0
}

setup_img() {
	[ $(id -u) -eq 0 ] || echo "must run this as root"

	LOOPDEV0=/dev/loop0
	LOOPDEV1=/dev/loop1
	/sbin/modprobe loop

	for ld in $LOOPDEV1 $LOOPDEV0; do
		for mnt in $(mount|fgrep $ld|cut -f3 -d' '); do
			umount -d $mnt || exit 1
			rmdir $mnt
		done
		losetup -a|fgrep -q $ld && losetup -d $ld
	done

	[ "$3" = "clean" ] && exit 0

	# create partition table with one active, bootable FAT16 partition
	# printf "n\np\n\n\n\na\n1\nt\n6\nw\n" | fdisk $1 >/dev/null 2>&1
	printf "o\nn\n\n\n\n\na\n1\nw\n" | fdisk -u -C64 -S63 -H16 $1 >/dev/null 2>&1

	# compute byte offset of partition
	start=$(fdisk -l $1|fgrep $1|tail -1|awk '{print $3}')
	size=$(fdisk -l $1|fgrep "Sector size"|awk '{print $7}')
	offset=$(expr $start \* $size)

	# set up loop devices for disk and partition
	losetup $LOOPDEV0 $1
	losetup -o $offset $LOOPDEV1 $LOOPDEV0

	# create filesystem on partition
	mkfs -t ext2 $LOOPDEV1 >/dev/null

	# mount filesystem
	MNTDIR=$(mktemp -d /tmp/osimg.XXXXXXXXXX)
	mount $LOOPDEV1 $MNTDIR

	mkdir -p $MNTDIR/boot/grub
	cat > $MNTDIR/boot/grub/device.map <<EOF
(hd0)   /dev/loop0
(hd0,1) /dev/loop1
EOF

	# install grub
	$GRUBDIR/sbin/grub-install --no-floppy --target=i386-pc\
		--grub-mkdevicemap=$MNTDIR/boot/grub/device.map\
		--root-directory=$MNTDIR $LOOPDEV0

	cp -a $2/boot $MNTDIR/boot

	umount $MNTDIR
	# repeat until losetup completes - might fail right after umount
	while ! losetup -d $LOOPDEV1; do sleep 1; umount $MNTDIR; done
	rmdir $MNTDIR
	while ! losetup -d $LOOPDEV0; do sleep 1; done

	exit 0
}

[ $# -lt 2 ] && {
	echo "usage: $0 <target> [<target-file>] <stagedir> <kernel binary> [gdb] [<module>] ..."
	echo "<target>: iso, pxe, usb, img"
	exit 1
}
target=$1; shift
if [ "$target" = "iso" -o "$target" = "usb" -o "$target" = "img" ]; then
	targetfile=$1; shift
elif [ "$target" != "pxe" ]; then
	echo $target not a valid target: iso, pxe, usb, img
	exit 1
fi
stage=$1; shift
kernel=$1; shift
mkdir -p $stage/boot/grub
cp $kernel $stage/boot
echo "set timeout=1" >> $stage/boot/grub/grub.cfg
echo "set default=0" >> $stage/boot/grub/grub.cfg
#echo "set pager=1" >> $stage/boot/grub/grub.cfg
#echo "set debug=all" >> $stage/boot/grub/grub.cfg
echo >> $stage/boot/grub/grub.cfg
echo 'menuentry "KOS" {' >> $stage/boot/grub/grub.cfg
echo -n "  multiboot2 /boot/$kernel" >> $stage/boot/grub/grub.cfg
echo -n " boot,cdi,dev,file,kmem,libc,lwip,perf,pci,proc,tests,threads,vm" >> $stage/boot/grub/grub.cfg
#echo -n " acpi" >> $stage/boot/grub/grub.cfg
if [ "$1" = "gdb" ]; then
	echo -n ",gdbe" >> $stage/boot/grub/grub.cfg
	shift 1
elif [ "$1" = "gdbdebug" ]; then
	echo -n ",gdbe,gdbd" >> $stage/boot/grub/grub.cfg
	shift 1
fi
echo >> $stage/boot/grub/grub.cfg
[ $# -gt 0 ] && cp $* $stage/boot && for i in $* ; do
	echo -n "  module2 /boot/" >> $stage/boot/grub/grub.cfg
	echo -n "$(basename $i)" >> $stage/boot/grub/grub.cfg
	echo " $(basename $i) sample args" >> $stage/boot/grub/grub.cfg
done
echo "  boot" >> $stage/boot/grub/grub.cfg
echo "}" >> $stage/boot/grub/grub.cfg
case $target in
	pxe)
		echo PXE boot files prepared in $stage;;
	usb) setup_usb $targetfile $stage;;
	img) setup_img $targetfile $stage;;
	iso)
		$GRUBDIR/bin/grub-mkrescue -o $targetfile $stage;;
esac
