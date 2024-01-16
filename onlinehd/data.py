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


def generate_weights_bin(path: str, model: OnlineHD) -> None :
    '''
    Dump the weights in a binary file. The file consists solely of
    the raw data
    '''
    # 10 classes 0-9
    # 784 features - pixel data
    # dim 4000 by default
    # model is classes x dim tensor
    with open(path, mode='wb') as f:
        f.write(model.model.numpy(force=True).tobytes())


def load_weights_bin(path: str, classes: int, dim: int) -> torch.Tensor:
    '''load weights bin into a classesxdim tensor assuming float32 dtype'''
    arr = torch.empty(classes, dim)
    with open(path, mode='rb') as f:
        for i in range(classes):
            for j in range(dim):
                b = f.read(4)
                if len(b) < 4:
                    break
                arr[i][j], = struct.unpack('f', b)
    return arr