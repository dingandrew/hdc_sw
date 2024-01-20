#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

typedef struct {
    int dim;
    int features;
    float* basis;
    float* base;
} Encoder;

Encoder* create_encoder(int features, int dim);


void destroy_encoder(Encoder* encoder);


void encoder_call(Encoder* encoder, float* x, int n, float* h);

void encoder_to(Encoder* encoder, int* args);
