#include "../includes/onlinehd.h"


int main(int argc, char const *argv[])
{
    OnlineHD *test = OnlineHD_init();


    size_t count = 0;
    float temp;

    //initalize data
    FILE *datax = fopen("../bin/x.bin", "rb");
    if (datax == NULL) {
        perror("Error opening file");
        exit(1);
    }
    while (fread(&temp, sizeof(float), 1, datax) == 1) {
        count++;
    }
    size_t rows = count / FEATURES;

    fclose(datax);
    float (*x)[DIM] = malloc(rows * sizeof(*x));
    if (x == NULL) {
        perror("Memory allocation failed");
        return 1;
    }
    fclose(datax);
    // Now read the data into the array
    datax = fopen("../bin/x.bin", "rb");
    if (datax == NULL) {
        perror("Error opening file");
        free(x);
        return 1;
    }
    for (size_t i = 0; i < rows; i++) {
        if (fread(x[i], sizeof(float), FEATURES, datax) != FEATURES) {
            if (feof(datax)) {
                break; // EOF reached, partial row read
            } else {
                perror("Error reading data");
                free(x);
                fclose(datax);
                return 1;
            }
        }
    }
    fclose(datax);
    FILE *fp2;
    fp2 = fopen("x.txt", "w");
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < FEATURES; j++) {
            fprintf(fp2, "%f ", x[i][j]);
        }
        putc('\n', fp2);
    }
    int n = rows;

    //initilize label 
    int y[n];
    FILE *labely = fopen("../bin/y.bin", "rb"); // Open the file in binary read mode
    if (labely == NULL) {
        perror("Error opening file");
        return 1;
    } 
    //load the data in
    size_t readItems = fread(y, sizeof(int), n, labely);
    if (readItems != n) {
        perror("Error reading label data");
        fclose(labely);
        return 1;
    }
    fclose(labely);
    
    //model
    FILE *model = fopen("../bin/weights.bin", "rb"); // Open the file in binary read mode
    if (model == NULL) {
        perror("Error opening file");
        return 1;
    } 
    //load the data in
    for (int i = 0; i < CLASSES; i++) {
        size_t readItems = fread(test->model[i], sizeof(float), DIM, model);
        if (readItems != DIM) {
            perror("Error reading model data");
            fclose(model);
            return 1;
        }
    }
    //check model
    FILE *fp;
    fp = fopen("weightsc.txt", "w");
    for (int i = 0; i < CLASSES; ++i) {
        for (int j = 0; j < DIM; ++j) {
            fprintf(fp, "%f ", test->model[i][j]);
        }
        putc('\n', fp);
    }
    fclose(model);
    //initialize other variable
    int batch_size = 10;
    int epochs = 5;
    float lr = 0.01; 
    float h[n][DIM];
    float dist[batch_size][CLASSES];
    int predictions[n];
    // //encode the data first 
    Encoder_encode(test->encoder,(float *)x,n,h);
    // //train the model
    OnlineHD_iterative_fit(test,(float *)x,h,n,y,lr,epochs,batch_size,dist,true);
    //get prediction 
    OnlineHD_call(test, (float *)x, n, h, dist, predictions, true);
    //test encoder
    // printf("Testing Encoder\n");
    // for (int i = 0; i < n; ++i) {
    //     printf("Encoded data point %d: ", i);
    //     for (int j = 0; j < DIM; ++j) {
    //         printf("encoded_output[%d][%d]: %f\n", i, j, h[i][j]);
    //     }
    //     printf("\n");
    // }
    //test cdist
    printf("Testing Cdist\n");
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < CLASSES; j++) {
            printf("Similarity between vector %d in x1 and vector %d in x2: %.8f\n", i, j, dist[i][j]);
        }
    }
    //test updated model
    // for (int i = 0; i < CLASSES; i++) 
    // {
    //     for (int j = 0; j < DIM; j++) 
    //     {
    //         printf("model is %f",test->model[i][j]);
    //     }
    // }
    // Calculate accuracy
    float accuracy = calculate_accuracy(y, predictions, n);
    printf("Accuracy of the model: %.2f%%\n", accuracy * 100);

    OnlineHD_free(test);
    free(x);
    return 0;
}
