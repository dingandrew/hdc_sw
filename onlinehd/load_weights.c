#include <stdio.h>


#define CLASSES 3
#define DIM     3

int main() {
    float weights[CLASSES][DIM];
    FILE *fp;
    fp = fopen("test.bin", "r");
    if (fp == NULL) {
        printf("failed to open file\n");
        return 1;
    }

    // read floats from bin
    for (int i = 0; i < CLASSES; ++i) {
        for (int j = 0; j < DIM; ++j) {
            // read float32
            int read = fread(&weights[i][j], 4, 1, fp);
            if (read != 1) {
                printf("error reading only read %d bytes\n", read);
                fclose(fp);
                return 1;
            }
        }
    }

    for (int i = 0; i < CLASSES; ++i) {
        for (int j = 0; j < DIM; ++j) {
            printf("%f ", weights[i][j]);
        }
        putc('\n', stdout);
    }
}