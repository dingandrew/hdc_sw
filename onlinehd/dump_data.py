from example import load
from data import generate_data_header


def main() -> None:
    x, x_test, y, y_test = load()
    generate_data_header('test.h', 4, x, y)

if __name__ == '__main__':
    main()