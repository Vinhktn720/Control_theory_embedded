#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
typedef struct {
	  float Q;  // Process noise (0.01-0.1)
	  float R;  // Measurement noise (1-10)
	  float P;  // Estimation error
	  float X;  // Estimated value
} KalmanFilter;
void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float P, float X);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);
#endif //KALMAN_FILTER_H
