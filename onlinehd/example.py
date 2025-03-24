import pathlib
from time import time

import torch
import sklearn.datasets
import sklearn.preprocessing
import sklearn.model_selection
import numpy as np

import onlinehd

from data import generate_bin_2d, generate_bin_1d

DIM = 128
CLASSES = 5
SAMPLES = 10

# loads simple mnist dataset
def load():
    # fetches data
    x, y = sklearn.datasets.fetch_openml('mnist_784', return_X_y=True)
    x = x.astype(np.float64)
    y = y.astype(np.int64)

    print("x", x)
    print("y", y)

    # split and normalize
    x, x_test, y, y_test = sklearn.model_selection.train_test_split(x, y)
    scaler = sklearn.preprocessing.Normalizer().fit(x)
    x = scaler.transform(x)
    x_test = scaler.transform(x_test)

    # changes data to pytorch's tensors
    x = torch.from_numpy(np.asarray(x)).float()
    y = torch.from_numpy(np.asarray(y)).long()
    x_test = torch.from_numpy(np.asarray(x_test)).float()
    y_test = torch.from_numpy(np.asarray(y_test)).long()
    print("x", x.size())
    print("y", y.size())
    print("xt", x_test.size())
    print("xt", y_test.size())
    return x, x_test, y, y_test

# simple OnlineHD training
def main():
    pathlib.Path('./bin').mkdir(exist_ok=True)
    print('Loading...')
    x, x_test, y, y_test = load()


    # Identify the unique classes in y
    unique_classes = torch.unique(y)

    # Select the first 5 classes
    first_5_classes = unique_classes[:CLASSES]
    
    # Create boolean masks for train and test sets
    mask_train = torch.isin(y, first_5_classes)
    mask_test = torch.isin(y_test, first_5_classes)

    # Filter to keep only the data from the first 5 classes
    x = x[mask_train]
    y = y[mask_train]
    x_test = x_test[mask_test]
    y_test = y_test[mask_test]
    
    # get datstet dimensions
    classes = y.unique().size(0)
    print("Classes: ", classes)
    if classes != CLASSES:
        print("WRONG CLASSES: ", classes)
    features = x.size(1)


    model = onlinehd.OnlineHD(CLASSES, features, DIM)

    print('Saving dataset samples...')
    x_view = x_test[0:SAMPLES]
    y_view = y_test[0:SAMPLES]
    generate_bin_2d('./bin/x.bin', x_view)
    generate_bin_1d('./bin/y.bin', y_view, 'i')

    if torch.cuda.is_available():
        x = x.cuda()
        y = y.cuda()
        x_test = x_test.cuda()
        y_test = y_test.cuda()
        model = model.to('cuda')
        print('Using GPU!')

    print('Training...')
    t = time()
    model = model.fit(x, y, bootstrap=1.0, lr=0.035, epochs=20)
    t = time() - t

    print('Validating...')
    yhat = model(x)
    yhat_test = model(x_test)
    acc = (y == yhat).float().mean()
    acc_test = (y_test == yhat_test).float().mean()
    print("ACCURACY", acc)
    print("ACCURACY TEST", acc_test)
    print("TIME", t)

    print("Saving model attributes...")
    generate_bin_2d("./bin/weights.bin", model.model)
    generate_bin_2d("./bin/encoder-basis.bin", model.encoder.basis)
    generate_bin_1d("./bin/encoder-base.bin", model.encoder.base)


if __name__ == '__main__':
    main()
