#include "../includes/onlinehd.h"

#define IMAGE_WIDTH 28
#define IMAGE_HEIGHT 28

void displayImage(float (*x)[FEATURES], int numImages) {
    if (x == NULL || numImages <= 0) {
        printf("Invalid input\n");
        return;
    }

    for (int imgIdx = 0; imgIdx < numImages; imgIdx++) {
        printf("Image %d:\n", imgIdx + 1);
        for (int i = 0; i < IMAGE_HEIGHT; i++) {
            for (int j = 0; j < IMAGE_WIDTH; j++) {
                int index = i * IMAGE_WIDTH + j;
                int pixelValue = (int)(x[imgIdx][index] * 255);

                // Display the pixel as an ASCII character (e.g., '#' for high intensity)
                if (pixelValue > 0) {
                    printf("#");
                } else {
                    printf(" ");
                }
            }
            printf("\n");
        }
        printf("\n");
    }
}


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
    
    printf("rows images: %ld \n", rows);


    fclose(datax);
    float (*x)[FEATURES] = malloc(rows * sizeof(*x));
    if (x == NULL) {
        perror("Memory allocation failed");
        return 1;
    }

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
    /////////////////////check x 
    // FILE *fp2;
    // fp2 = fopen("x.txt", "w");
    // for (int i = 0; i < rows; i++) {
    //     for (int j = 0; j < FEATURES; j++) {
    //         fprintf(fp2, "%f ", x[i][j]);
    //     }
    //     putc('\n', fp2);
    // }

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


    //////////////////////////////check y
    // FILE *fp4;
    // fp4 = fopen("y.txt", "w");
    // for (int i = 0; i < rows; i++) {
    //         fprintf(fp4, "%d ", y[i]);
    //     putc('\n', fp4);
    // }


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

    fseek(model, 0, SEEK_END); // Seek to the end of the file
    long file_size = ftell(model); // Get the current position,
    if (file_size % sizeof(float) == 0) {
        // The file size is a multiple of the size of a float (4 bytes)
        int num_floats = file_size / sizeof(float);
        printf("model size: %ld bytes (%d floats)\n", file_size, num_floats);
    } else {
        printf("model size is not a multiple of the size of a float.\n");
    }
    
    /////////////////////////check model
    FILE *fp;
    fp = fopen("weightsc.txt", "w");
    for (int i = 0; i < CLASSES; ++i) {
        for (int j = 0; j < DIM; ++j) {
            fprintf(fp, "%f ", test->model[i][j]);
        }
        putc('\n', fp);
    }
    fclose(model);

    //display images
    displayImage(x, 5);

    //initialize other variable
    int batch_size = 5;
    int epochs = 100;
    float lr = 0.01; 
    float h[n][DIM];
    float dist[batch_size][CLASSES];
    int predictions[n];

    // //encode the data first 
    Encoder_encode(test->encoder,x,n,h);


    //////////////////////check encoded 
    FILE *fp5;
    fp5 = fopen("h.txt", "w");
    for (int i = 0; i < rows; i++) {
        fprintf(fp5,"Data number %d\n\n\n\n\n\n",i);
        for (int j = 0; j < DIM; j++) {
            fprintf(fp5, "%f ", h[i][j]);
        }
        putc('\n', fp5);
    }


    // //train the model
    //OnlineHD_iterative_fit(test,(float *)x,h,n,y,lr,epochs,batch_size,dist,true); // DO NOT RETRAIN MODEL
    //get prediction 
    OnlineHD_call(test, x, n, h, dist, predictions, true);
    
    ////////check model
    // FILE *fp3;
    // fp3 = fopen("modelafter.txt", "w");
    // for (int i = 0; i < CLASSES; ++i) {
    //     for (int j = 0; j < DIM; ++j) {
    //         fprintf(fp3, "%f ", test->model[i][j]);
    //     }
    //     putc('\n', fp3);
    // }
    // fclose(model);
    //////////////////////////// test encoder
    // printf("Testing Encoder\n");
    // for (int i = 0; i < n; ++i) {
    //     printf("Encoded data point %d: ", i);
    //     for (int j = 0; j < DIM; ++j) {
    //         printf("encoded_output[%d][%d]: %f\n", i, j, h[i][j]);
    //     }
    //     printf("\n");
    // }

    ///////////////////////// test cdist
    // printf("Testing Cdist\n");
    // for (int i = 0; i < n; i++) {
    //     for (int j = 0; j < CLASSES; j++) {
    //         printf("Similarity between vector %d in x1 and vector %d in x2: %.8f\n", i, j, dist[i][j]);
    //     }
    // }

    /////////////////////// y data
    // for (int j = 0; j < (sizeof(y)/sizeof(y[0])); j++) 
    for (int j = 0; j < 5; j++) 
    {
        printf("y[%d] is %d \n",j, y[j]);
    }

    /////////////////////// predictions data
    // for (int j = 0; j < (sizeof(predictions)/sizeof(predictions[0])); j++) 
    for (int j = 0; j < 5; j++) 
    {
        printf("predictions[%d] is %d \n",j, predictions[j]);
    }

    float accuracy = calculate_accuracy(y, predictions, n);
    printf("Accuracy of the model: %.2f%%\n", accuracy * 100);

    OnlineHD_free(test);
    free(x);
    return 0;
}
