# onlinehd c version

## Quick start

1. Run the example.py and make sure the bin folder is under hdc_sw. 
2. cd to the c_src folder 
3. type make to generate a prewritten test 
4. type ./test or test to run the test sample 

## data change 
1. In the test_main.c, input data is x with size of x[][FEATURES] and the data is inputted from the bin file
2. model data is with size model[CLASSES][DIM] and the model data from the bin file
3. You can change the data size and model data by directly changing the path of the bin file
4. The test sample prints out the cdist and the accuracy of the prediction.