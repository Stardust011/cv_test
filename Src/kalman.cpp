#include "kalman.h"
//using namespace AntiKalman;

Point2f AntiKalman::KalmanFilter::Anti_KalmanFilter(Point2f currentPoint, Point2f kalmanPoint, double anti_range)
{
	double size_x = 680;
//	double size_y=480;
	Point2f anti_kalmanPoint;
/*	if ((currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) <= size_x
		|| (currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) >= 0||
		(currentPoint.y + anti_range*(currentPoint.y - kalmanPoint.y)) <= size_y
		|| (currentPoint.y + anti_range*(currentPoint.y - kalmanPoint.y)) >= 0)//Prevent Anti-kal out of Mat
	{
		if (abs(currentPoint.x - kalmanPoint.x) > 3||
		abs(currentPoint.y - kalmanPoint.y) > 3)//When points are closed, no Anti-kalman to reduce shaking
		//	return anti_kalmanPoint.x = currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x);
			return anti_kalmanPoint = Point2f((currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)), currentPoint.y+ anti_range*(currentPoint.y - kalmanPoint.y));
		else
			return anti_kalmanPoint = Point2f(currentPoint.x,currentPoint.y);
	}
	else
	{
	//	return anti_kalmanPoint.x = currentPoint.x;
		return anti_kalmanPoint = Point2f(currentPoint.x, currentPoint.y);
	}*/
	
	if ((currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) <= size_x
		|| (currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) >= 0)//Prevent Anti-kal out of Mat
	{
		if (abs(currentPoint.x - kalmanPoint.x) > 3)//When points are closed, no Anti-kalman to reduce shaking
		//	return anti_kalmanPoint.x = currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x);
			return anti_kalmanPoint = Point2f((currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)), currentPoint.y);
		else
			return anti_kalmanPoint = Point2f(currentPoint.x,currentPoint.y);
	}
	else
	{
	//	return anti_kalmanPoint.x = currentPoint.x;
		return anti_kalmanPoint = Point2f(currentPoint.x, currentPoint.y);
	}
	
	
}

int main1()
{
	float size_x = 640;//cols of side
	float size_y = 480;//rows of side
	float x = 20;//CV_32F: float
	float y = 240;//CV_32F: float
	int color = 0;//gradually varied color
	float anti_range = 0.5;//**Larger, anti-kalman more radical
	Mat image(size_y, size_x, CV_8UC3);
	
	AntiKalman::KalmanFilter kf(x, y);//Differ from cv::KalmanFilter
	Point2f currentPoint(x, y);
	Point2f kalmanPoint(x, y);
	Point2f anti_kalmanPoint(x,y);
	while (1)
	{
		//image = Scalar(0,0,0);//Clear points before
		currentPoint = Point2f(x, y);
		kalmanPoint = kf.run(x, y);
	//	cout << "currentPoint:" << currentPoint << endl;
	//	cout << "kalmanPoint:" << kalmanPoint << endl;
	//	cout << "anti_kalmanPoint:" << anti_kalmanPoint << endl;
		anti_kalmanPoint = kf.Anti_KalmanFilter(currentPoint, kalmanPoint, 0.5);
	/*	if ((currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) <= size_x
			|| (                               .x + anti_range*(currentPoint.x - kalmanPoint.x)) >= 0)//Prevent Anti-kal out of Mat
		{
			if (abs(currentPoint.x - kalmanPoint.x) > 3)//When points are closed, no Anti-kalman to reduce shaking
				anti_kalmanPoint.x = currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x);
			else
				anti_kalmanPoint.x = currentPoint.x;
		}
		else
		{
			anti_kalmanPoint.x = currentPoint.x;
		}
		*/
		//y is the same.
		
		circle(image, anti_kalmanPoint, 3, Scalar(0, 255, 0 + color), 2);//predicted point with green
		circle(image, currentPoint, 3, Scalar(255, 0, 0 + color), 2);//current position with red
		imshow("Anti-KalmanPoint", image);
		cout << "Current: " << currentPoint << " Kalman: " << kalmanPoint << " Anti-Kalman: " << anti_kalmanPoint << endl;
		x += 105;
	//  y+=10;
		color += 20;
		waitKey(1000);
		if(x==335)//Initialize Kalman Filter when losing target in a long time
		{
			kf.ResetKalmanFilter(x, y);
		}
//		if (color >= 255)
//		{
//			color = 255;
//		}
		if ((x >= size_x) || (y >= size_y) || (x <= 0) || (y <= 0))
		{
//			imwrite("Anti-KalmanPoint.jpg", image);
			break;
		}
	}
	waitKey(0);
	return 0;
}
