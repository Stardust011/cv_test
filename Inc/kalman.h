#ifndef _KALMAN_H
#define _KALMAN_H
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace AntiKalman
{
	class KalmanFilter
	{
	public:
		KalmanFilter(int x, int y) :
			KF_(4, 2)
			/*
			KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F )
			"dynamParams = 4": 4*1 vector of state (x, y, delta x, delta y)
			"measureParams = 2": 2*1 vector of measurement (x, y)
			*/
		{
			measurement_ = Mat::zeros(2, 1, CV_32F);// (x, y)
			KF_.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,//**Latter 1: Larger, faster regression
				0, 1, 0, 1,//**Latter 1: Larger, faster regression
				0, 0, 1, 0,
				0, 0, 0, 1);
			setIdentity(KF_.measurementMatrix, Scalar::all(1));
			setIdentity(KF_.processNoiseCov, Scalar::all(1e-10));//**10: Larger, slower regression
//			setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-1));//1: Larger, quicker regression
			setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-8));//1: Larger, quicker regression
			setIdentity(KF_.errorCovPost, Scalar::all(1));
			KF_.statePost = (Mat_<float>(4, 1) << x, y, 0, 0);//Ensure beginner is default value
		}

		void ResetKalmanFilter(int x, int y)
		{
			cout << "Reset KalmanFilter" << endl;
			KF_.statePost = (Mat_<float>(4, 1) << x, y, 0, 0);//Ensure beginner is default value
		}

		Point2f run(float x, float y)
		{
			Mat prediction = KF_.predict();
			Point2f predict_pt = Point2f(prediction.at<float>(0), prediction.at<float>(1));

			measurement_.at<float>(0, 0) = x;
			measurement_.at<float>(1, 0) = y;
			KF_.correct(measurement_);
			return predict_pt;
		}
		Point2f Anti_KalmanFilter(Point2f currentPoint, Point2f kalmanPoint, double anti_range);
	private:
		Mat measurement_;
		cv::KalmanFilter KF_;//Differ from Kalman_example::KalmanFilter
	};
}


#endif
