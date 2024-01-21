#include "../includes/spatial.h"


// Function to calculate cosine similarity
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
//testing
/*
int main() {
    // Specify the dimensions
    int n1 = 14; // You can change this as needed
    int n2 = 20; // You can change this as needed

    // Initialize x1 and x2 with random values
    float x1[n1][DIM];
    float x2[n2][DIM];
    srand(time(NULL));
    for (int i = 0; i < n1; i++)
        for (int j = 0; j < DIM; j++)
            x1[i][j] = (float)rand() / RAND_MAX;

    for (int i = 0; i < n2; i++)
        for (int j = 0; j < DIM; j++)
            x2[i][j] = (float)rand() / RAND_MAX;

    // output array
    float dist[n1][n2];

    // Calculate cosine similarity
    cos_cdist(x1, n1, x2, n2, dist);

    // Print the resulting similarity matrix
    for (int i = 0; i < n1; i++) {
        for (int j = 0; j < n2; j++) {
            printf("Similarity between vector %d in x1 and vector %d in x2: %.8f\n", i, j, dist[i][j]);
        }
    }

    return 0;
}
*/