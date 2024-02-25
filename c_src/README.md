# onlinehd c version for vicuna

## Prerequisites

1. Build the riscv toolchain
```
git clone https://github.com/riscv-collab/riscv-gnu-toolchain.git
cd riscv-gnu-toolchain
./configure --prefix=[install directory] --enable-multilib --with-arch=rv32imav --with-abi=ilp32
make
```
2. Add the [install directory]/bin to path

## Quick start

1. Run the example.py and make sure the bin folder is under hdc_sw. 
2. cd to the c_src folder 
3. run `make` to build
4. run `make sim` to run the simulation

## data change 
1. In the test_main.c, input data is x with size of x[][FEATURES] and the data is inputted from the bin file
2. model data is with size model[CLASSES][DIM] and the model data from the bin file
3. You can change the data size and model data by directly changing the path of the bin file
4. The test sample prints out the cdist and the accuracy of the prediction.