#include "../includes/encoder.h"

void create_encoder(Encoder *encoder) 
{
    srand(time(NULL));
    //set up the inital encode 
    for (int i = 0; i < DIM * FEATURES; ++i) {
        encoder->basis[i] = (float)rand() / RAND_MAX * 2.0 - 1.0;
    }
    for (int i = 0; i < DIM; ++i) {
        encoder->base[i] = (float)rand() / RAND_MAX * 2.0 * M_PI;
    }
}
//encode: the encoder created; x: input data; n: number of data points; h:output
void encoder_call(Encoder* encoder, float* x, int n, float h[][DIM]) 
//encode: the encoder created; x: input data; n: number of data points; h:output
{
    int bsize = ceil(0.01 * n);
    float temp[bsize * DIM];

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
                    temp[j * DIM + k] += x[(i + j) * FEATURES + l] * encoder->basis[k * FEATURES + l];
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

// might not be useful right now
// void encoder_to(Encoder* encoder, int* args) {
//     free(encoder->basis);
//     free(encoder->base);
//     encoder->basis = malloc(encoder->dim * encoder->features * sizeof(float));
//     encoder->base = malloc(encoder->dim * sizeof(float));
// }


//used for testing can be deleted
// int main()
// {
//     // Sample test data
//     float data[3][FEATURES];
//     // Seed the random number generator
//     srand(time(NULL));
//     // Generate random data
//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < FEATURES; ++j) {
//             data[i][j] = (float)rand() / RAND_MAX;  // Generate a float between 0 and 1
//         }
//     }
//     int num_data_points = sizeof(data) / sizeof(data[0]);

//     // Initialize the encoder
//     Encoder *encoder = malloc(sizeof(Encoder));
//     if (encoder == NULL) {
//         fprintf(stderr, "Failed to allocate memory for encoder\n");
//         return 1;
//     }
//     create_encoder(encoder);
    
//     // created encoded output
//     float encoded_output[num_data_points][DIM];
//     // Encode the data
//     encoder_call(encoder,(float *) data, num_data_points, encoded_output);
//     for (int i = 0; i < num_data_points; ++i) {
//         printf("Encoded data point %d: ", i);
//         for (int j = 0; j < DIM; ++j) {
//             printf("encoded_output[%d][%d]: %f\n", i, j, encoded_output[i][j]);
//         }
//         printf("\n");
//     }
//     free(encoder);
//     return 0;
// }
