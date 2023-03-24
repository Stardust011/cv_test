#pragma once
#ifndef _BUFWU_H
#define _BUFWU_H

#include "opencv2/core/core.hpp"
//#include "opencv2/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
//#include "opencv2/ml.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>
//#include <opencv2/core/bufferpool.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>
//#include "new_uart_thread_init.h"
using namespace std;
using namespace cv;
const static double PI = 3.1415926;
#define ERROR_POINT1 Point(640,640)

class Ligth {
public:
    Ligth(string);

    ~Ligth();

    void Ligth_HSVimage_deal(Mat &, Mat &);

    Mat Ligth_morpho_deal(Mat &);

    Mat Ligth_Pic_Erode_deal(Mat &);

    void Dafu_Out(Mat &, bool, Rect &, double &, Point2f &);

    Mat Ligth_Pic_deal(Mat &);

    Point2f Point_Get(Point2f &, Point2f &, double &);

    void display(string &, const Mat &);

    void findCircle2(Point2f &, Point2f &, Point2f &, Point2f &, double &);

    void Circle_Center(Point2f, Point2f, double &, Point2f &);

    double Point_Distance(Point2f &, Point2f &);
    //Point Get_Result_Point();
private:
    double R;
    int math;
    Point2f P_C[1000], P_Center[1000];
    Point2f Get_Center, CenTer;

};

#endif
#pragma once
