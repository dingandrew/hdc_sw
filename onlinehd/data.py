import torch
from onlinehd import OnlineHD


CLASSES = 10
FEATURES = 784
DIM = 4000


def generate_data_header(path: str, n: int, x: torch.Tensor, y: torch.Tensor) -> None :
    '''Generate C array declarations with n images and labels'''
    # 52500 samples
    # x: img data         52500 by 784 matrix
    # y: labels on data   52500 element vector
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


def generate_weights_header(model: OnlineHD) -> None :
    # 10 classes 0-9
    # 784 features - pixel data
    # dim 4000 by default
    # model is classes x dim tensor
    ...
