#include <math.h>

float fasthd_cos(float* x, float* xs, float* out, int x_size, int xs_size);

float* onepass(float* x, float* y, float* m, int n, int c, float lr, int x_size);