#include "kalman_filter.h"

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float P, float X){
	kf->Q =Q;
	kf->R =R;
	kf->P =P;
	kf->X =X;
}
float KalmanFilter_Update(KalmanFilter *kf, float measurement){
	  kf->P += kf->Q;
	  float K = kf->P / (kf->P + kf->R);
	  kf->X += K * (measurement - kf->X);
	  kf->P *= (1 - K);
	  return kf->X;
}
