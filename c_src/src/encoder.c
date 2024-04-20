#include "../includes/encoder.h"
#include <riscv_vector.h>

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

    size_t vlmax = __riscv_vsetvlmax_e32m1();  // Set maximum vector length for 32-bit single precision

    for (int i = 0; i < n; i += bsize) 
    {
        int current_batch_size = fmin(bsize, n - i);
        // Reset temp to zero for each batch
        for (int j = 0; j < current_batch_size * DIM; j++) 
        {
            temp[j] = 0.0;
            
        }

        // Matrix multiplication
        // for (int j = 0; j < current_batch_size; j++) 
        // {
        //     for (int k = 0; k < DIM; k++) 
        //     {
        //         for (int l = 0; l < FEATURES; l++) 
        //         {
        //             temp[j * DIM + k] += x[i + j][l] * encoder->basis[k * FEATURES + l];
        //         }
        //     }
        // }
        
        // Matrix multiplication with RVV Intrincs
        for (int j = 0; j < current_batch_size; j++) {
            for (int k = 0; k < DIM; k++) {
                float *ptr_x = &x[i + j][0];
                float *ptr_basis = &encoder->basis[k * FEATURES];
                int l = FEATURES;
                vfloat32m1_t vec_s = __riscv_vfmv_v_f_f32m1(0.0f, vlmax); // Initialize the sum vector to zero
                vfloat32m1_t vec_zero = __riscv_vfmv_v_f_f32m1(0.0f, vlmax);
                for (size_t vl; l > 0; l -= vl, ptr_x += vl, ptr_basis += vl) {
                    vl = __riscv_vsetvl_e32m1(l); // Set the vector length for the remaining elements
                    
                    vfloat32m1_t vec_x = __riscv_vle32_v_f32m1(ptr_x, vl);
                    vfloat32m1_t vec_basis = __riscv_vle32_v_f32m1(ptr_basis, vl);
                    
                    vec_s = __riscv_vfmacc_vv_f32m1(vec_s, vec_x, vec_basis, vl);
                }
                
                vfloat32m1_t vec_sum;
                vec_sum = __riscv_vfredusum_vs_f32m1_f32m1(vec_s, vec_zero, vlmax);
                float sum = __riscv_vfmv_f_s_f32m1_f32(vec_sum);
                temp[j * DIM + k] = sum;
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


//used for testing can be deleted
/*
int main()
{
    // Sample test data
    float data[3][FEATURES];
    // Seed the random number generator
    srand(time(NULL));
    // Generate random data
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < FEATURES; ++j) {
            data[i][j] = (float)rand() / RAND_MAX;  // Generate a float between 0 and 1
        }
    }
    int num_data_points = sizeof(data) / sizeof(data[0]);

    // Initialize the encoder
    Encoder *encoder = malloc(sizeof(Encoder));
    if (encoder == NULL) {
        fprintf(stderr, "Failed to allocate memory for encoder\n");
        return 1;
    }
    create_encoder(encoder);
    //set up the inital encode 
    for (int i = 0; i < DIM * FEATURES; ++i) {
        printf("Basis is %f\n",encoder->basis[i]); 
    }
    for (int i = 0; i < DIM; ++i) {
        printf("Basis is %f\n",encoder->base[i]); 
    }
    // created encoded output
    float encoded_output[num_data_points][DIM];
    // Encode the data
    // encoder_call(encoder,(float *) data, num_data_points, encoded_output);
    // for (int i = 0; i < num_data_points; ++i) {
    //     printf("Encoded data point %d: ", i);
    //     for (int j = 0; j < DIM; ++j) {
    //         printf("encoded_output[%d][%d]: %f\n", i, j, encoded_output[i][j]);
    //     }
    //     printf("\n");
    // }
    free(encoder);
    return 0;
}
*/