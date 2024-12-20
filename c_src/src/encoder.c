#include "../includes/encoder.h"
// #include <riscv_vector.h>

void create_encoder(Encoder *encoder) 
{
    FILE *Basis = fopen("../bin/encoder-basis.bin", "rb"); // Open the file in binary read mode
    if (Basis == NULL) {
        perror("Error opening file");
        return;
    }

    // Read data for encoder->basis
    size_t readItems = fread(encoder->basis, sizeof(float), DIM * FEATURES, Basis);
    if (readItems != DIM * FEATURES) {
        perror("Error reading basis data");
        fclose(Basis);
        return;
    }

    fclose(Basis); // Close the file

    // Open the file for encoder->base
    FILE *Base = fopen("../bin/encoder-base.bin", "rb");
    if (Base == NULL) {
        perror("Error opening base file");
        return;
    }

    // Read data for encoder->base
    size_t readItemsBase = fread(encoder->base, sizeof(float), DIM, Base);
    if (readItemsBase != DIM) {
        perror("Error reading base data");
        fclose(Base);
        return;
    }

    fclose(Base); // Close the base file
}
//encode: the encoder created; x: input data; n: number of data points; h:output
void encoder_call(Encoder* encoder, float x[][FEATURES], int n, float h[][DIM]) 
//encode: the encoder created; x: input data; n: number of data points; h:output
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
                    temp[j * DIM + k] += x[i + j][l] * encoder->basis[k * FEATURES + l];
                }
            }
        }
        

        // Applying cosine and sine functions
        for (int j = 0; j < current_batch_size; j++) 
        {
            for (int k = 0; k < DIM; k++) 
            {
                float dot_product = temp[j * DIM + k];
                float base_value = encoder->base[k];
                h[i + j][k] = cos(dot_product + base_value) * sin(dot_product);
            }
        }
    }
}

