#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

#define FEATURES 784
#define DIM 4000

typedef struct {
    float basis[DIM * FEATURES];
    float base[DIM];
} Encoder;

void create_encoder(Encoder *encoder);

void destroy_encoder(Encoder* encoder);


void encoder_call(Encoder* encoder, float* x, int n, float h[][DIM]);
void encoder_to(Encoder* encoder, int* args);
