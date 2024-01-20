
#include "../includes/encoder.h"

Encoder* create_encoder(int features, int dim) {
    Encoder* encoder = malloc(sizeof(Encoder));
    // Error handling 
    if (encoder == NULL) 
    {
        return NULL; 
    }
    encoder->dim = dim;
    encoder->features = features;
    encoder->basis = malloc(dim * features * sizeof(float));
    encoder->base = malloc(dim * sizeof(float));

    // Random initialization
    srand(time(NULL));
    //for basis (by standard deviation)
    for (int i = 0; i < dim * features; ++i) 
    {
        encoder->basis[i] = (float)rand() / RAND_MAX * 2.0 - 1.0; // Normal distribution approximation
    }
    //for base (by 0-2pi)
    for (int i = 0; i < dim; ++i) 
    {
        encoder->base[i] = (float)rand() / RAND_MAX * 2.0 * M_PI;
    }
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
    // Handle memory allocation error
    if (temp == NULL) 
    {
        return;
    }
    // for (int i = 0; i < n; i += bsize) {
    //     for (int j = 0; j < bsize; j++) {
    //         for (int k = 0; k < encoder->features; k++) {
    //             temp[j * encoder->dim + k] = 0.0;
    //             for (int l = 0; l < encoder->dim; l++) {
    //                 temp[j * encoder->dim + k] += x[(i + j) * encoder->features + l] * encoder->basis[l * encoder->features + k];
    //             }
    //         }
    //     }
        
    //     for (int j = 0; j < bsize; j++) {
    //         for (int k = 0; k < encoder->dim; k++) {
    //             h[(i + j) * encoder->dim + k] = temp[j * encoder->dim + k] + encoder->base[k];
    //             h[(i + j) * encoder->dim + k] = cos(h[(i + j) * encoder->dim + k]) * sin(temp[j * encoder->dim + k]);
    //         }
    //     }
    // }
    
    // free(temp);
    for (int i = 0; i < n; i += bsize) 
    {
        int current_batch_size = fmin(bsize, n - i);
        // Reset temp to zero for each batch
        for (int j = 0; j < current_batch_size * encoder->dim; j++) 
        {
            temp[j] = 0.0;
        }
        // Matrix multiplication
        for (int j = 0; j < current_batch_size; j++) 
        {
            for (int k = 0; k < encoder->dim; k++) 
            {
                for (int l = 0; l < encoder->features; l++) 
                {
                    temp[j * encoder->dim + k] += x[(i + j) * encoder->features + l] * encoder->basis[k * encoder->features + l];
                }
            }
        }

        // Applying cosine and sine functions
        for (int j = 0; j < current_batch_size; j++) 
        {
            for (int k = 0; k < encoder->dim; k++) 
            {
                float dot_product = temp[j * encoder->dim + k];
                h[(i + j) * encoder->dim + k] = cos(dot_product + encoder->base[k]) * sin(dot_product);
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

// int main()
// {
//     // Sample test data
//     float test_input[][2] = {{1.0, 2.0}, {3.0, 4.0}}; // 2D input data
//     int num_data_points = sizeof(test_input) / sizeof(test_input[0]);
//     int input_features = 2; // Number of features in the input data
//     int encoded_dim = 6;    // Dimensionality of the encoded space

//     // Initialize the encoder
//     Encoder* my_encoder = create_encoder(input_features, encoded_dim);
//     if (my_encoder == NULL) {
//         fprintf(stderr, "Failed to create encoder\n");
//         return 1;
//     }

//     // Allocate memory for the encoded output
//     float* encoded_output = (float*)malloc(num_data_points * encoded_dim * sizeof(float));
//     if (encoded_output == NULL) {
//         fprintf(stderr, "Failed to allocate memory for encoded output\n");
//         destroy_encoder(my_encoder);
//         return 1;
//     }

//     // Encode the data
//     encoder_call(my_encoder, (float*)test_input, num_data_points, encoded_output);

//     // Print the encoded output
//     for (int i = 0; i < num_data_points; ++i) {
//         printf("Encoded data point %d: ", i);
//         for (int j = 0; j < encoded_dim; ++j) {
//             printf("%f ", encoded_output[i * encoded_dim + j]);
//         }
//         printf("\n");
//     }

//     // Clean up
//     free(encoded_output);
//     destroy_encoder(my_encoder);

//     return 0;
// }
