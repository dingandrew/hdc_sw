import torch

from example import load
import onlinehd

from data import generate_bin_2d, load_bin_2d, generate_bin_1d, load_bin_1d

# simple OnlineHD training
def main():
    print('Loading...')
    x, x_test, y, y_test = load()
    classes = y.unique().size(0)
    features = x.size(1)
    model = onlinehd.OnlineHD(classes, features)

    if torch.cuda.is_available():
        x = x.cuda()
        y = y.cuda()
        x_test = x_test.cuda()
        y_test = y_test.cuda()
        model = model.to('cuda')
        print('Using GPU!')

    print("Loading model attributes...")
    load_bin_2d("weights.bin", model.model)
    load_bin_2d("encoder-basis.bin", model.encoder.basis)
    load_bin_1d("encoder-base.bin", model.encoder.base)
    
    print('Validating...')
    yhat = model(x)
    yhat_test = model(x_test)
    acc = (y == yhat).float().mean()
    acc_test = (y_test == yhat_test).float().mean()
    print(f'{acc = :6f}')
    print(f'{acc_test = :6f}')


if __name__ == '__main__':
    main()
