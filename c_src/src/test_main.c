#include "../includes/onlinehd.h"


int main(int argc, char const *argv[])
{
    OnlineHD *test = OnlineHD_init();

    // // Define and initialize input data x and an array to store predictions
    // float x[/* number of data points */][DIM] = { /* data points */ };
    // int predictions[/* number of data points */];

    // // Predict classes for the input data
    // OnlineHD_predict(&model, x, /* number of data points */, predictions);

    // // Output the predictions
    // for (int i = 0; i < /* number of data points */; i++) {
    //     printf("Prediction for data point %d: %d\n", i, predictions[i]);
    // }
    float x[50][DIM];
    int y[50];
    //input x
    for (int i = 0; i < 50; i++)
    {
        for (int j = 0; j < DIM; j++)
        {
            x[i][j] = (float)rand() / RAND_MAX;
            y[i] = rand() % CLASSES; // Random label
        }
    }
    int n = sizeof(x) / sizeof(x[0]);
    //model 
    for (int i = 0; i < CLASSES; i++) 
    {
        for (int j = 0; j < DIM; j++) 
        {
            test->model[i][j] = (float)rand() / RAND_MAX;
        }
    }
    //initialize 
    int batch_size = 10;
    int epochs = 5;
    float lr = 0.01; 
    float h[n][DIM];
    float dist[batch_size][CLASSES];
    int predictions[n];
    //encode the data first 
    Encoder_encode(test->encoder,(float *)x,n,h);
    //train the model
    OnlineHD_iterative_fit(test,(float *)x,h,n,y,lr,epochs,batch_size,dist,false);
    //check encoder
    // printf("Testing Encoder\n");
    // for (int i = 0; i < n; ++i) {
    //     printf("Encoded data point %d: ", i);
    //     for (int j = 0; j < DIM; ++j) {
    //         printf("encoded_output[%d][%d]: %f\n", i, j, h[i][j]);
    //     }
    //     printf("\n");
    // }
    //check cdist
    // printf("Testing Cdist\n");
    // for (int i = 0; i < n; i++) {
    //     for (int j = 0; j < CLASSES; j++) {
    //         printf("Similarity between vector %d in x1 and vector %d in x2: %.8f\n", i, j, dist[i][j]);
    //     }
    // }
    // check maximum score 
    // printf("Testing OnlineHD_Call\n");
    // for (int i = 0; i < n; i++) 
    // {
    //     printf("%d\n",predictions[i]);
    // }
    // for (int i = 0; i < CLASSES; i++) 
    // {
    //     for (int j = 0; j < DIM; j++) 
    //     {
    //         printf("model is %f",test->model[i][j]);
    //     }
    // }

    OnlineHD_free(test);
    return 0;
}
