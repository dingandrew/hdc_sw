# onlinehd c version

## Quick start

1. cd to the c_src folder 
2. type make to generate a prewritten test 
3. type ./test or test to run the test sample 

## data change 
1. In the test_main.c, input data is x with size of x[3][DIM] and the data is randomized. 
2. model data is with size model[CLASSES][DIM] and the model data is randomized 
3. You can change the data size and model data by directly changing its size and data. 
4. The test sample prints out the encoded_output data, Cdist between the output data and the model, and the index of the highest scores. 