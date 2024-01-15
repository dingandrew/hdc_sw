#include "spatial.h"

float cos_cdist(float* x1, float* x2, int size, float eps) {
    float norms1 = 0.0;
    float norms2 = 0.0;
    float cdist = 0.0;

    for (int i = 0; i < size; i++) {
        norms1 += x1[i] * x1[i];
        norms2 += x2[i] * x2[i];
    }

    norms1 = sqrt(norms1) + eps;
    norms2 = sqrt(norms2) + eps;

    for (int i = 0; i < size; i++) {
        cdist += x1[i] * x2[i];
    }

    cdist /= norms1;
    cdist /= norms2;

    return cdist;
}