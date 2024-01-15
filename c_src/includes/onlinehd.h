#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "encoder.h"
#include "spatial.h"


typedef struct {
    int classes;
    int dim;
    Encoder encoder;
    float* model;
} OnlineHD;

OnlineHD* OnlineHD_init(int classes, int features, int dim);

int OnlineHD_call(OnlineHD* onlineHD, float* x, bool encoded);

int OnlineHD_predict(OnlineHD* onlineHD, float* x, bool encoded);

float* OnlineHD_probabilities(OnlineHD* onlineHD, float* x, bool encoded);

float* OnlineHD_scores(OnlineHD* onlineHD, float* x, bool encoded);

float* Encoder_encode(Encoder* encoder, float* x);

void OnlineHD_fit(OnlineHD* onlineHD, float* x, float* y, bool encoded, float lr, int epochs, int batch_size, bool one_pass_fit, float bootstrap);


void OnlineHD_one_pass_fit(OnlineHD* onlineHD, float* h, float* y, float lr, float bootstrap);

void OnlineHD_onepass(float* h, float* y, float* model, float lr, int* onePassIdxs, int onePassIdxsSize);

void OnlineHD_iterative_fit(OnlineHD* onlineHD, float* h, float* y, float lr, int epochs, int batch_size);

void OnlineHD_to(OnlineHD* onlineHD, ...);

Encoder* Encoder_init(int features, int dim);

float* Encoder_encode(Encoder* encoder, float* x);

void Encoder_free(Encoder* encoder);


