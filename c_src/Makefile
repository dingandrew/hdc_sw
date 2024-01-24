all: test

test: test_main.o onlinehd.o encoder.o spatial.o
	gcc -o test test_main.o onlinehd.o encoder.o spatial.o -lm

encoder.o: src/encoder.c includes/encoder.h 
	gcc -c src/encoder.c

spatial.o: src/spatial.c includes/spatial.h
	gcc -c src/spatial.c

onlinehd.o: src/onlinehd.c includes/encoder.h includes/onlinehd.h includes/spatial.h
	gcc -c src/onlinehd.c

test_main.o: src/test_main.c includes/onlinehd.h includes/encoder.h includes/spatial.h
	gcc -c src/test_main.c

clean:
	rm *.exe *.o