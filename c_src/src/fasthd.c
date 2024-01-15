#include "fasthd.h"

float cos(float* x, float* xs, float* out, int x_size, int xs_size) {
    // Calculate dot product
    float dot_product = 0.0;
    for (int i = 0; i < x_size; i++) {
        dot_product += x[i] * xs[i];
    }
    
    // Calculate norms
    float x_norm = 0.0;
    float xs_norm = 0.0;
    for (int i = 0; i < x_size; i++) {
        x_norm += x[i] * x[i];
        xs_norm += xs[i] * xs[i];
    }
    x_norm = sqrt(x_norm);
    xs_norm = sqrt(xs_norm);
    
    // Calculate cosine similarity
    float similarity = dot_product / (x_norm * xs_norm);
    
    // Store the result in out
    *out = similarity;
    
    return similarity;
}

float* onepass(float* x, float* y, float* m, int n, int c, float lr, int x_size) {
    float* scores = new float[c];
    for (int i = 0; i < n; i++) {
        float* spl = &x[i * x_size];
        int lbl = y[i];
        
        // Calculate cosine similarity
        cos(spl, m, scores, x_size, c);
        
        // Find the index of the minimum score
        int prd = 0;
        float min_score = scores[0];
        for (int j = 1; j < c; j++) {
            if (scores[j] < min_score) {
                prd = j;
                min_score = scores[j];
            }
        }
        
        // Update the model
        for (int j = 0; j < x_size; j++) {
            m[lbl * x_size + j] += lr * spl[j] * scores[lbl];
            m[prd * x_size + j] -= lr * spl[j] * scores[prd];
        }
    }
    
    return m;
}
