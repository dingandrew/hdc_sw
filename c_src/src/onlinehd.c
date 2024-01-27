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
//use this directly as predict
//onlineHD: the object; x:input data; n:number of data points; h:encoded data; dist:cos sim; predictions: output prediction label
void OnlineHD_call(OnlineHD* onlineHD, float* x,int n,float h[][DIM], float dist[][CLASSES], int predictions[n], bool encoded) {
    OnlineHD_scores(onlineHD,(float *)x,n,h,dist,encoded);

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
        //printf("max index is %d \n",predictions[i] = maxIndex);
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
//onlineHD: the object; x:input data; n:number of data points; h:encoded data; dist:cos sim;
void OnlineHD_scores(OnlineHD* onlineHD, float* x, int n, float h[][DIM], float dist[][CLASSES], bool encoded) 
{
    if (!encoded)
    {
        //encode the data
        printf("Encoding\n");
        Encoder_encode(onlineHD->encoder, x,n ,h);
        //calculate the cos similarity 
        cos_cdist(h, n, onlineHD->model, CLASSES, dist);
    }
    else //dont encode just calculate the score
    {
        cos_cdist(h, n, onlineHD->model, CLASSES, dist);
    }
}

void OnlineHD_iterative_fit(OnlineHD* onlineHD, float* x, float h[][DIM],int n, int y[], float lr, int epochs, int batch_size, float dist[][CLASSES], bool encoded) 
{
    float batchData[batch_size][DIM]; // Temporary array to store the current batch data
    for (int epoch = 0; epoch < epochs; epoch++) 
    {
        for (int i = 0; i < n; i += batch_size) 
        {
            int end = i + batch_size > n ? n : i + batch_size;
            int batchSize = end - i;
            // Copy data for the current batch into batchData
            for (int j = 0; j < batchSize; j++) {
                for (int k = 0; k < DIM; k++) {
                    batchData[j][k] = h[i + j][k];
                    //printf("The data is %f \n",batchData[j][k] );
                }
            }
            // Calculate scores for each batch
            OnlineHD_scores(onlineHD,(float *)x,n,batchData,dist,true);

            // Arrays to store predicted labels and other parameters for each batch
            int y_pred[batchSize];
            bool wrong[batchSize];
            float alpha1[batchSize];
            float alpha2[batchSize];

            for (int j = 0; j < batchSize; j++) 
            {
                // Find predicted label
                y_pred[j] = 0;
                for (int k = 1; k < CLASSES; k++) 
                {
                    if (dist[j][k] > dist[j][y_pred[j]]) 
                    {
                        y_pred[j] = k;
                        
                    }
                }

                // Determine if the prediction is wrong
                wrong[j] = y[i + j] != y_pred[j];

                // Calculate alpha1 and alpha2
                alpha1[j] = 1.0 - dist[j][y[i + j]];
                alpha2[j] = dist[j][y_pred[j]] - 1.0;
            }

            // Update model for each label
            for (int lbl = 0; lbl < CLASSES; lbl++) 
            {
                for (int j = 0; j < batchSize; j++) 
                {
                    if (wrong[j]) 
                    {
                        if (y[i + j] == lbl) 
                        {
                            for (int k = 0; k < DIM; k++) 
                            {
                                onlineHD->model[lbl][k] += lr * alpha1[j] * h[i + j][k];
                                //printf("update is %d\n",k);
                            }
                        }
                        if (y_pred[j] == lbl) 
                        {
                            for (int k = 0; k < DIM; k++) 
                            {
                                onlineHD->model[lbl][k] += lr * alpha2[j] * h[i + j][k];
                            }
                        }
                    }
                }
            }
        }
    }
}


// void OnlineHD_to(OnlineHD* onlineHD, ...) {
//     // convert model and encoder to new device
// }

float calculate_accuracy(int* true_labels, int* predictions, int n) {
    int correct = 0;
    for (int i = 0; i < n; i++) {
        if (true_labels[i] == predictions[i]) {
            correct++;
        }
    }
    return (float)correct / n; // Return the proportion of correct predictions
}

void OnlineHD_free(OnlineHD* onlineHD) {
    free(onlineHD->encoder);
    free(onlineHD);
}

Encoder* Encoder_init() {
    Encoder *encoder = malloc(sizeof(Encoder));
    create_encoder(encoder);
    return(encoder);
}
//onlineHD: the object; x:input data; n:number of data points; h:encoded data
void Encoder_encode(Encoder* encoder, float* x, int n, float h[][DIM]) 
{
    // encode x
    encoder_call(encoder,(float *) x, n, h);
}

