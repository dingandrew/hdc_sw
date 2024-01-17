#include <stdio.h>


#define CLASSES 10
#define DIM     4000


int load_bin(char *path, char *arr, size_t data_size, size_t n);
    

int main() {
    float weights[CLASSES][DIM];
    if (load_bin("./bin/weights.bin", (char*)weights, 4, CLASSES*DIM) == 1) {
        return 1;
    }
    FILE *fp;
    fp = fopen("weightsc.txt", "w");
    for (int i = 0; i < CLASSES; ++i) {
        for (int j = 0; j < DIM; ++j) {
            fprintf(fp, "%f ", weights[i][j]);
        }
        putc('\n', fp);
    }
}


int load_bin(char *path, char *arr, size_t data_size, size_t n) {
    FILE *fp;
    fp = fopen(path, "r");
    if (fp == NULL) {
        printf("failed to open file\n");
        return 1;
    }

    // read floats from bin
    for (int i = 0; i < n; ++i) {
        // read float32
        int read = fread(&arr[i * data_size], data_size, 1, fp);
        if (read != 1) {
            printf("error: only read %d floats of %ld\n", i - 1, n);
            fclose(fp);
            return 1;
        }
    }
    fclose(fp);
}