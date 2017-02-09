#/bin/bash
for i in $*; do
	echo "extern int $i();" >> UserMain.h
done
echo >> UserMain.h
echo "static void UserMain() {" >> UserMain.h
for i in $*; do
	echo "  $i();" >> UserMain.h
done
echo "}" >> UserMain.h
