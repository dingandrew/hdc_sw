# hdc_sw

Setup the enviroment:

python -m venv env

pip install -r requirements.txt


## To Run the HDC C-Model

### DATA CONVERTER

First run `example.py` to train and save the model and a few test vectors.
Running `example_load.py` will load the saved model and test it on the saved test vectors. The model weights are also put into a file `weightspy.txt`
Compile and run `load_weights.c`
```
gcc -o load_weights load_weights.c
./load_weights
```
This will load from the saved weights and output as text to `weightsc.txt` which should be identical to `weightspy.txt`

### Compile C Model



## To Run the Vicuna Sanity Test

source env_setup.sh

cd vicuna/test

make mul COMPILER=gcc

