#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define FEATURES 784
#define CLASSES 10
#define DIM 4000

typedef struct {
    float basis[DIM * FEATURES];
    float base[DIM];
    float model[CLASSES][DIM];
} OnlineHD;


void OnlineHD_Top(OnlineHD onlineHD, float x[][FEATURES],int n,float h[][DIM], float dist[][CLASSES], int predictions[n]);//bool encoded

void cos_cdist(float x1[][DIM], int n1, float x2[][DIM], int n2, float dist[][n2]);

void encoder_call(OnlineHD onlineHD, float x[][FEATURES], int n, float h[][DIM]);
