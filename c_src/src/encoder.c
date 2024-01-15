

#include "encoder.h"

Encoder* create_encoder(int features, int dim) {
    Encoder* encoder = malloc(sizeof(Encoder));
    encoder->dim = dim;
    encoder->features = features;
    encoder->basis = malloc(dim * features * sizeof(float));
    encoder->base = malloc(dim * sizeof(float));
    return encoder;
}

void destroy_encoder(Encoder* encoder) {
    free(encoder->basis);
    free(encoder->base);
    free(encoder);
}

void encoder_call(Encoder* encoder, float* x, int n, float* h) {
    int bsize = ceil(0.01 * n);
    float* temp = malloc(bsize * encoder->dim * sizeof(float));
    
    for (int i = 0; i < n; i += bsize) {
        for (int j = 0; j < bsize; j++) {
            for (int k = 0; k < encoder->features; k++) {
                temp[j * encoder->dim + k] = 0.0;
                for (int l = 0; l < encoder->dim; l++) {
                    temp[j * encoder->dim + k] += x[(i + j) * encoder->features + l] * encoder->basis[l * encoder->features + k];
                }
            }
        }
        
        for (int j = 0; j < bsize; j++) {
            for (int k = 0; k < encoder->dim; k++) {
                h[(i + j) * encoder->dim + k] = temp[j * encoder->dim + k] + encoder->base[k];
                h[(i + j) * encoder->dim + k] = cos(h[(i + j) * encoder->dim + k]) * sin(temp[j * encoder->dim + k]);
            }
        }
    }
    
    free(temp);
}

void encoder_to(Encoder* encoder, int* args) {
    free(encoder->basis);
    free(encoder->base);
    encoder->basis = malloc(encoder->dim * encoder->features * sizeof(float));
    encoder->base = malloc(encoder->dim * sizeof(float));
}
