#include "../includes/onlinehd.h"

OnlineHD* OnlineHD_init() {
    OnlineHD* onlineHD = malloc(sizeof(OnlineHD));
    onlineHD->encoder = Encoder_init(FEATURES, DIM);
    // Initialize the model matrix with zeros
    for (int i = 0; i < CLASSES; i++) {
        for (int j = 0; j < DIM; j++) {
            onlineHD->model[i][j] = 0.0f;
        }
    }
    return onlineHD;
}

void OnlineHD_call(OnlineHD* onlineHD, float* x,int n,float h[][DIM], float dist[][CLASSES], int predictions[n]) {
    OnlineHD_scores(onlineHD,(float *)x,n,h,dist);

    for (int i = 0; i < n; i++) {
        int maxIndex = 0;
        float maxScore = dist[i][0];
        for (int j = 1; j < CLASSES; j++) {
            if (dist[i][j] > maxScore) {
                maxScore = dist[i][j];
                maxIndex = j;
            }
        }
        predictions[i] = maxIndex;
        
    }
}

// int OnlineHD_predict(OnlineHD* onlineHD, float* x, bool encoded) {
//     return OnlineHD_call(onlineHD, x, encoded);
// }

// float* OnlineHD_probabilities(OnlineHD* onlineHD, float* x, bool encoded) {
//     float* scores = OnlineHD_scores(onlineHD, x, encoded);
//     float* probabilities = malloc(onlineHD->classes * sizeof(float));
//     float sum = 0.0;
//     for (int i = 0; i < onlineHD->classes; i++) {
//         probabilities[i] = expf(scores[i]);
//         sum += probabilities[i];
//     }
//     for (int i = 0; i < onlineHD->classes; i++) {
//         probabilities[i] /= sum;
//     }
//     free(scores);
//     return probabilities;
// }

void OnlineHD_scores(OnlineHD* onlineHD, float* x, int n, float h[][DIM], float dist[][CLASSES]) 
{
    //encode the data 
    Encoder_encode(onlineHD->encoder, x,n ,h);
    //calculate the cos similarity 
    cos_cdist(h, n, onlineHD->model, CLASSES, dist);
}

// void OnlineHD_fit(OnlineHD* onlineHD, float* x, float* y, bool encoded, float lr, int epochs, int batch_size, bool one_pass_fit, float bootstrap) {
//     float* h = encoded ? x : Encoder_encode(onlineHD->encoder, x);
//     if (one_pass_fit) {
//         OnlineHD_one_pass_fit(onlineHD, h, y, lr, bootstrap);
//     }
//     OnlineHD_iterative_fit(onlineHD, h, y, lr, epochs, batch_size);
//     free(h);
// }

// void OnlineHD_one_pass_fit(OnlineHD* onlineHD, float* h, float* y, float lr, float bootstrap) {
//     if (bootstrap == 'single-per-class') {
//         int* idxs = malloc(onlineHD->classes * sizeof(int));
//         for (int i = 0; i < onlineHD->classes; i++) {
//             idxs[i] = y[i] == i;
//         }
//         int banned = idxs[0];
//         for (int i = 1; i < onlineHD->classes; i++) {
//             if (idxs[i] > banned) {
//                 banned = idxs[i];
//             }
//         }
//         for (int i = 0; i < onlineHD->dim; i++) {
//             onlineHD->model[banned * onlineHD->dim + i] += h[i] * lr;
//         }
//         free(idxs);
//     } else {
//         int cut = ceil(bootstrap * onlineHD->dim);
//         float* h_ = malloc(cut * sizeof(float));
//         float* y_ = malloc(cut * sizeof(float));
//         for (int i = 0; i < cut; i++) {
//             h_[i] = h[i];
//             y_[i] = y[i];
//         }
//         for (int lbl = 0; lbl < onlineHD->classes; lbl++) {
//             for (int i = 0; i < cut; i++) {
//                 if (y_[i] == lbl) {
//                     for (int j = 0; j < onlineHD->dim; j++) {
//                         onlineHD->model[lbl * onlineHD->dim + j] += h_[i * onlineHD->dim + j] * lr;
//                     }
//                 }
//             }
//         }
//         int* banned = malloc(cut * sizeof(int));
//         for (int i = 0; i < cut; i++) {
//             banned[i] = i;
//         }
//         int n = onlineHD->dim;
//         bool* todo = malloc(n * sizeof(bool));
//         for (int i = 0; i < n; i++) {
//             todo[i] = true;
//         }
//         for (int i = 0; i < cut; i++) {
//             todo[banned[i]] = false;
//         }
//         float* h_ = malloc(n * sizeof(float));
//         float* y_ = malloc(n * sizeof(float));
//         int* onePassIdxs = malloc(n * sizeof(int));
//         int onePassIdxsSize = 0;
//         for (int i = 0; i < n; i++) {
//             if (todo[i]) {
//                 h_[onePassIdxsSize] = h[i];
//                 y_[onePassIdxsSize] = y[i];
//                 onePassIdxs[onePassIdxsSize] = i;
//                 onePassIdxsSize++;
//             }
//         }
//         OnlineHD_onepass(h_, y_, onlineHD->model, lr, onePassIdxs, onePassIdxsSize);
//         free(banned);
//         free(todo);
//         free(h_);
//         free(y_);
//         free(onePassIdxs);
//     }
// }

// void OnlineHD_onepass(float* h, float* y, float* model, float lr, int* onePassIdxs, int onePassIdxsSize) {
//     for (int i = 0; i < onePassIdxsSize; i++) {
//         int idx = onePassIdxs[i];
//         int lbl = y[idx];
//         for (int j = 0; j < onlineHD->dim; j++) {
//             model[lbl * onlineHD->dim + j] += h[idx * onlineHD->dim + j] * lr;
//         }
//     }
// }

// void OnlineHD_iterative_fit(OnlineHD* onlineHD, float* h, float* y, float lr, int epochs, int batch_size) {
//     int n = onlineHD->dim;
//     for (int epoch = 0; epoch < epochs; epoch++) {
//         for (int i = 0; i < n; i += batch_size) {
//             int batchSize = i + batch_size < n ? batch_size : n - i;
//             float* h_ = malloc(batchSize * sizeof(float));
//             float* y_ = malloc(batchSize * sizeof(float));
//             for (int j = 0; j < batchSize; j++) {
//                 h_[j] = h[i + j];
//                 y_[j] = y[i + j];
//             }
//             float* scores = OnlineHD_scores(onlineHD, h_, true);
//             int* y_pred = malloc(batchSize * sizeof(int));
//             for (int j = 0; j < batchSize; j++) {
//                 int maxIndex = 0;
//                 for (int k = 1; k < onlineHD->classes; k++) {
//                     if (scores[j * onlineHD->classes + k] > scores[j * onlineHD->classes + maxIndex]) {
//                         maxIndex = k;
//                     }
//                 }
//                 y_pred[j] = maxIndex;
//             }
//             bool* wrong = malloc(batchSize * sizeof(bool));
//             for (int j = 0; j < batchSize; j++) {
//                 wrong[j] = y_[j] != y_pred[j];
//             }
//             int* aranged = malloc(batchSize * sizeof(int));
//             for (int j = 0; j < batchSize; j++) {
//                 aranged[j] = j;
//             }
//             float* alpha1 = malloc(batchSize * sizeof(float));
//             float* alpha2 = malloc(batchSize * sizeof(float));
//             for (int j = 0; j < batchSize; j++) {
//                 alpha1[j] = 1.0 - scores[j * onlineHD->classes + y_[j]];
//                 alpha2[j] = scores[j * onlineHD->classes + y_pred[j]] - 1.0;
//             }
//             for (int lbl = 0; lbl < onlineHD->classes; lbl++) {
//                 for (int j = 0; j < batchSize; j++) {
//                     if (y_[j] == lbl) {
//                         for (int k = 0; k < onlineHD->dim; k++) {
//                             onlineHD->model[lbl * onlineHD->dim + k] += lr * alpha1[j] * h_[j * onlineHD->dim + k];
//                         }
//                     }
//                     if (y_pred[j] == lbl) {
//                         for (int k = 0; k < onlineHD->dim; k++) {
//                             onlineHD->model[lbl * onlineHD->dim + k] += lr * alpha2[j] * h_[j * onlineHD->dim + k];
//                         }
//                     }
//                 }
//             }
//             free(h_);
//             free(y_);
//             free(scores);
//             free(y_pred);
//             free(wrong);
//             free(aranged);
//             free(alpha1);
//             free(alpha2);
//         }
//     }
// }

// void OnlineHD_to(OnlineHD* onlineHD, ...) {
//     // convert model and encoder to new device
//     //testing git branch, will be changed tomorrow 
//     // if (newType == onlineHD->currentType) {
//     //     // No change needed
//     //     return;
//     // }

//     // if (newType == double) {
//     //     // Convert from float to double
//     //     double* newModel = (double*)malloc(onlineHD->classes * onlineHD->dim * sizeof(double));
//     //     for (int i = 0; i < onlineHD->classes * onlineHD->dim; ++i) {
//     //         newModel[i] = (double)onlineHD->model[i];
//     //     }
//     //     free(onlineHD->model);
//     //     onlineHD->model = newModel;
//     //     onlineHD->currentType = double;
//     // } else if (newType == float) {
//     //     // Convert from double to float
//     //     float* newModel = (float*)malloc(onlineHD->classes * onlineHD->dim * sizeof(float));
//     //     for (int i = 0; i < onlineHD->classes * onlineHD->dim; ++i) {
//     //         newModel[i] = (float)onlineHD->model[i];
//     //     }
//     //     free(onlineHD->model);
//     //     onlineHD->model = newModel;
//     //     onlineHD->currentType = float;
//     // }

//     // Also convert the encoder, if necessary
//     // ...
// }

// void OnlineHD_free(OnlineHD* onlineHD) {
//     Encoder_free(onlineHD->encoder);
//     free(onlineHD);
// }

Encoder* Encoder_init() {
    Encoder *encoder = malloc(sizeof(Encoder));
    create_encoder(encoder);
    return(encoder);
}

void Encoder_encode(Encoder* encoder, float* x, int n, float h[][DIM]) 
{
    // encode x
    encoder_call(encoder,(float *) x, n, h);
}

