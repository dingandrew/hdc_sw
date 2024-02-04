# hdc_sw

Setup the enviroment:

python -m venv env

pip install -r requirements.txt


## To Run the Vicuna Sanity Test

source env_setup.sh

cd vicuna/test

make mul COMPILER=gcc

