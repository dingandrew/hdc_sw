import torch
from onlinehd import OnlineHD
import struct


CLASSES = 10
FEATURES = 784
DIM = 4000


def generate_data_header(path: str, n: int, x: torch.Tensor, y: torch.Tensor) -> None :
    '''Generate C array declarations with n images and labels'''
    # 52500 samples
    # x: img data         52500 by 784 matrix float
    # y: labels on data   52500 element vector long
    if n > len(x) or n > len(y):
        raise ValueError("n too big")
    with open(path, mode='w') as f:
        # images
        f.write(f'float x[{n}][{FEATURES}] = {{\n')
        for i in range(n):
            f.write('  {')
            for j in range(FEATURES):
                f.write(f'{x[i][j]:8f},')
            f.write('},\n')
        f.write('};\n')

        # labels
        f.write('char y[] = { ')
        for i in range(n):
            f.write(f'{y[i]}, ')
        f.write('};\n')


def generate_bin_2d(path: str, arr: torch.Tensor) -> None :
    '''Dump a 2d tensor of floats in a binary file. The file consists solely of
    the raw data'''
    # 10 classes 0-9
    # 784 features - pixel data
    # dim 4000 by default
    # model is classes x dim tensor
    with open(path, mode='wb') as f:
        for row in arr:
            for n in row:
                f.write(struct.pack('f', n))


def load_bin_2d(path: str, arr: torch.Tensor) -> None:
    '''load a 2d tensor bin into `arr` assuming float32 dtype. The file should
    be the raw data'''
    rows, cols = arr.shape
    with open(path, mode='rb') as f:
        for i in range(rows):
            for j in range(cols):
                b = f.read(4)
                if len(b) < 4:
                    break
                arr[i][j], = struct.unpack('f', b)

def generate_bin_1d(path: str, arr: torch.Tensor) -> None:
    '''Dump a 1d tensor of floats in a binary file. The file consists solely of
    the raw data'''
    with open(path, mode='wb') as f:
        for n in arr:
            f.write(struct.pack('f', n))

def load_bin_1d(path: str, arr: torch.Tensor) -> None:
    '''load a 1d tensor bin into `arr` assuming float32 dtype. The file should
    be the raw data'''
    with open(path, mode='rb') as f:
        for i in range(*arr.shape):
            b = f.read(4)
            if len(b) < 4:
                break
            arr[i], = struct.unpack('f', b)