#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "encoder.h"
#include "spatial.h"
//#include "fasthd.h"
#define CLASSES 10
#define DIM 4000
#define FEATRUES 748

#include <stdio.h>
#include <stdlib.h>

typedef struct {
    Encoder *encoder;
    float model[CLASSES][DIM];
} OnlineHD;

OnlineHD* OnlineHD_init();

void OnlineHD_call(OnlineHD* onlineHD, float x[][FEATURES],int n,float h[][DIM], float dist[][CLASSES], int predictions[n], bool encoded);//bool encoded

//int OnlineHD_predict(OnlineHD* onlineHD, float* x,int n,float h[][DIM], float dist[][CLASSES]); //bool encoded

//float* OnlineHD_probabilities(OnlineHD* onlineHD, float* x, bool encoded);

void OnlineHD_scores(OnlineHD* onlineHD, float x[][FEATURES], int n, float h[][DIM], float dist[][CLASSES], bool encoded);


void OnlineHD_iterative_fit(OnlineHD* onlineHD, float* x, float h[][DIM],int n, int y[], float lr, int epochs, int batch_size, float dist[][CLASSES], bool encoded);
//void OnlineHD_to(OnlineHD* onlineHD, ...);

Encoder* Encoder_init();

void Encoder_encode(Encoder* encoder, float x[][FEATURES], int n, float h[][DIM]) ;

void OnlineHD_free(OnlineHD* onlineHD);

float calculate_accuracy(int* true_labels, int* predictions, int n);
