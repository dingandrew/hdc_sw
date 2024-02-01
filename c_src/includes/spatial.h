#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#define DIM 4000
#define FEATURES 784

void cos_cdist(float x1[][DIM], int n1, float x2[][DIM], int n2, float dist[][n2]);