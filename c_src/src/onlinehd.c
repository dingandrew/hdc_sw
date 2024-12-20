#include "../includes/onlinehd.h"



/**
    onlineHD: the object; 
    x:input data; 
    n:number of data points; 
    h:encoded data; 
    dist:cos sim; 
    predictions: output prediction label
*/
void OnlineHD_Top(OnlineHD onlineHD, float x[][FEATURES],int n,float h[][DIM], float dist[][CLASSES], int predictions[n]) {
    
    // Encode test data
    encoder_call(onlineHD, x , n, h);
    cos_cdist(h, n, onlineHD.model, CLASSES, dist);

    for (int i = 0; i < n; i++) {
        int maxIndex = 0;
        float maxScore = dist[i][0];
        for (int j = 1; j < CLASSES; j++) {
            if (dist[i][j] > maxScore) {
                maxScore = dist[i][j];
                maxIndex = j;
            }
        }
        predictions[i] = maxIndex;
    }
}



/**
    encode: the encoder created; 
    x: input data; 
    n: number of data points; 
    h:output
*/
void encoder_call(OnlineHD onlineHD, float x[][FEATURES], int n, float h[][DIM]) 
{
    int bsize = ceil(0.01 * n);
    float temp[bsize * DIM];

    // size_t vlmax = __riscv_vsetvlmax_e32m1();  // Set maximum vector length for 32-bit single precision

    for (int i = 0; i < n; i += bsize) 
    {
        int current_batch_size = fmin(bsize, n - i);
        // Reset temp to zero for each batch
        for (int j = 0; j < current_batch_size * DIM; j++) 
        {
            temp[j] = 0.0;
            
        }

        // Matrix multiplication
        for (int j = 0; j < current_batch_size; j++) 
        {
            for (int k = 0; k < DIM; k++) 
            {
                for (int l = 0; l < FEATURES; l++) 
                {
                    temp[j * DIM + k] += x[i + j][l] * onlineHD.basis[k * FEATURES + l];
                }
            }
        }
        

        // Applying cosine and sine functions
        for (int j = 0; j < current_batch_size; j++) 
        {
            for (int k = 0; k < DIM; k++) 
            {
                float dot_product = temp[j * DIM + k];
                float base_value = onlineHD.base[k];
                h[i + j][k] = cos(dot_product + base_value) * sin(dot_product);
            }
        }
    }
}

void cos_cdist(float x1[][DIM], int n1, float x2[][DIM], int n2, float dist[][n2]) {
    float eps = 1e-8;

    for (int i = 0; i < n1; i++) {
        for (int j = 0; j < n2; j++) {
            float dot_product = 0.0;
            float norm_x1 = 0.0;
            float norm_x2 = 0.0;

            // Compute dot product and norms
            for (int k = 0; k < DIM; k++) {
                dot_product += x1[i][k] * x2[j][k];
                norm_x1 += x1[i][k] * x1[i][k];
                norm_x2 += x2[j][k] * x2[j][k];
            }

            norm_x1 = sqrtf(norm_x1) + eps;
            norm_x2 = sqrtf(norm_x2) + eps;

            dist[i][j] = dot_product / (norm_x1 * norm_x2);
        }
    }
}