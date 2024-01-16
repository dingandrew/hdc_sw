import torch

from example import load
import onlinehd

from data import generate_bin_2d, load_bin_2d, generate_bin_1d, load_bin_1d

# simple OnlineHD training
def main():
    print('Loading...')
    x = torch.empty(5, 784, dtype=torch.float)
    load_bin_2d('./bin/x.bin', x)
    y = torch.empty(5, dtype=torch.int)
    load_bin_1d('./bin/y.bin', y, 'i')
    classes = 10
    features = 784
    model = onlinehd.OnlineHD(classes, features)

    if torch.cuda.is_available():
        x = x.cuda()
        y = y.cuda()
        model = model.to('cuda')
        print('Using GPU!')

    print("Loading model attributes...")
    load_bin_2d("./bin/weights.bin", model.model)
    load_bin_2d("./bin/encoder-basis.bin", model.encoder.basis)
    load_bin_1d("./bin/encoder-base.bin", model.encoder.base)
    
    print('Validating...')
    yhat = model(x)
    acc = (y == yhat).float().mean()
    print(f'{acc = :6f}')


if __name__ == '__main__':
    main()
