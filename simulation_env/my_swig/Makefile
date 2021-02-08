_example.so : example.o example_wrap.o
	gcc -shared example.o example_wrap.o -o _example.so -lpython3.5m

example.o : example.c
	gcc -c -fPIC -I/usr/include/python3.5m example.c

example_wrap.o : example_wrap.c
	gcc -c -fPIC -I/usr/include/python3.5m example_wrap.c

example_wrap.c example.py : example.i example.h
	swig -python example.i

clean:
	rm -f *.o *.so example_wrap.* example.py*

test:
	python3 test.py

all: _example.so test

.PHONY: clean test all

.DEFAULT_GOAL := all