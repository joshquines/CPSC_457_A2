CXXFLAGS="-D__KOS__ -std=c++11 -iquote .. -iquote ../include -g -O3"
echo "marray - expected vs actual:
25 12 99 3 8"
g++ $CXXFLAGS marray.cc 
echo $(echo 17 25 33 12 42 -1 2 0 4 -1 99 -1 | ./a.out)
echo "bitmap - expected vs actual:
empty yes no not empty 5 100 500 0"
g++ $CXXFLAGS -mpopcnt bitmap.cc || exit 1
echo $(echo 100 200 300 400 500 -1 200 250 | ./a.out)
rm a.out
echo "hierbitmap - expected vs actual:
empty yes no not empty 2000 5000"
g++ $CXXFLAGS -mpopcnt hierbitmap.cc || exit 1
echo $(echo 1000 2000 3000 400 5000 -1 400 1000 -1 3000 2222 | ./a.out)
rm a.out
echo "region - expected vs actual:
15-20 30-40 50-60 15-20 30 40 50-60 15-20 50-60 15-20 50-55 not out out 15-55"
g++ $CXXFLAGS -mpopcnt region.cc || exit 1
echo $(echo i 10 20 i 30 40 i 50 60 r 10 15 r 25 45 r 30 40 r 55 60 o 10 20 o 70 80 i 18 55| ./a.out)
rm a.out
