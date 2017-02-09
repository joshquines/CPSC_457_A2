#!/bin/bash
source $(dirname $0)/../config
brctl addbr br0
ifconfig br0 inet 192.168.57.1/24 up
echo "allow br0" > $TOOLSDIR/etc/qemu/bridge.conf
echo "arp -s 192.168.57.200 525400123456"
echo "arp -s 192.168.57.201 525400123457 # for testing non-matching packets"
exit 0
