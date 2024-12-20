all: test


python_model: onlinehd/example.py onlinehd/example_load.py load_weights
	python onlinehd/example.py
	python onlinehd/example_load.py
	./load_weights
	@if cmp -s weightspy.txt weightsnotme.txt; then \
		echo "PASS: Exported C Weights and Python Model are identical"; \
	else \
		echo "FAIL: Exported C Weights are different? :0"; \
	fi

load_weights: onlinehd/load_weights.c
	gcc -o load_weights onlinehd/load_weights.c

test: test_main.o onlinehd.o 
	gcc -g -std=c99 -o test test_main.o onlinehd.o -lm

onlinehd.o: c_src/src/onlinehd.c c_src/includes/onlinehd.h 
	gcc -g -std=c99 -c c_src/src/onlinehd.c

test_main.o: c_src/src/test_main.c c_src/includes/onlinehd.h 
	gcc -g -std=c99 -c c_src/src/test_main.c

clean:
	rm test load_weights *.o *.txt