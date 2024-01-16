from example import load
from data import generate_data_header, generate_weights_bin, load_weights_bin
import torch


def main() -> None:
    x, x_test, y, y_test = load()
    generate_data_header('test.h', 2, x, y)
    class FakeModel:
        def __init__(self):
            self.model = torch.tensor([[1.1, 1.2, 1.3], [2.1, 2.2, 2.3], [3.1, 3.2, 3.3]])
    fake_model = FakeModel()
    generate_weights_bin('test.bin', fake_model)
    arr = load_weights_bin('test.bin', 3, 3)
    print(arr)


if __name__ == '__main__':
    main()