# g++ -c thirdso.cpp
# ar crv libthirdso.a thirdso.o
# g++ main.cpp -L ./ -lthirdso -o main
# ./main

g++ -c -g hook.cpp
ar crv libhook.a hook.o
g++ -g main.cpp -L ./ -lhook -ldl -lthirdso -o main
./main