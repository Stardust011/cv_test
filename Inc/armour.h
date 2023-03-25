#pragma once
#ifndef _ARMOUR_H
#define _ARMOUR_H

#include "config.h"

#include <iostream>
#include <fstream>

#include <unistd.h>
#include <dirent.h>


using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

#include <cmath>

#ifndef WORK_WITH_WINDOWS

#include <semaphore.h>

#endif

#include "new_uart_thread_init.h"
#include "led.h"
#include "kalman.h"
#include "BufWu.h"

#include <logger.h>
#include <V4l2Device.h>
#include <V4l2Capture.h>

//#include "opencv_v4l2.h"

//#include "BigWindCar.h"
#define ERROR_POINT Point(640,640)
#define ERROR_POINT_BUF Point2f(640,640)


bool BIG_OR_SMALL = false;

//using namespace AntiKalman;

enum MODE_FLAG {
    MODE_NONE = 0,
    MODE_RED = 1,
    MODE_BLUE = 2

};

struct combine_rect_height {
    Rect contour_rect;
    int height;
};

extern int CMD_COLOR;

class armour {
public:
    double cost_time;

    static void *collect_pic_thread(void *arg);

    armour(int);

    ~armour();

    void Port_armour();

    vector<vector<Point> > get_point_contours(const Mat &);//3
    Mat erode_pic(const Mat &, int);

    Mat dilate_pic(const Mat &, int);

    void get_rect_pic_contour(Mat, const vector<vector<Point> > &, vector<vector<Point> > &);//4
    vector<double> get_rotate_angle(const vector<vector<Point> > &);

    vector<double> get_rotate_angle(Mat &, const vector<vector<Point> > &);

    vector<Scalar> cal_distance_theata(const vector<double> &);

    vector<Scalar> judge_the_theata(const vector<Scalar> &, double);

    void get_rotate_angle(const vector<vector<Point> > &, vector<double> &);//
    //void get_rotate_angle(const vector<vector<Point> > &, vector<double>&);
    void set_the_rect(const Mat &, const vector<Scalar> &, const vector<vector<Point> > &, vector<Rect> &, int, int);

    Point select_the_rect(Mat &, vector<Rect> &, int &, Rect &);

    Point select_the_rect_area(Mat &, const vector<Rect> &);

    Mat rect_the_pic(const Mat &, Mat &, vector<Rect> &);

    Mat rgb_to_gray(const Mat &, int, int, int, int, int, int, int);

    Mat YUV_to_gray(const Mat &, int, int, int, int, int, int, int);

    Mat HSV_to_gray(const Mat &, int, int, int, int, int, int, int);

    void fire(Mat &);

    vector<vector<Point> > judge_the_min_distance(const Mat &, const vector<vector<Point> > &, double, double);

    vector<Rect> bubble_sort(vector<double> distance, vector<Rect> distance_point);

    double get_max_area(vector<Rect> rect_vector);

    // below func is build for write image
    int return_the_exsit_num(string);

    string change_int_into_path_jpg(string, int);

    double CalculateZ(float, float, float, float);

//	double CalculateZ(Rect armour_rect);
//    double CalculateZ(Rect);

    vector<double> Calculate_angle(const Point &, const Point &, double);

    Point select_the_rect(Mat &, vector<Rect> &);

//	Point select_the_rect_hero(Mat& ,vector<Rect>& ,int & );
    void select_rect_by_angle(const Rect &select_rect, vector<double> &angle_vector, double distance);

    Point select_the_rect_rotate(Mat &, vector<Rect> &, int &, Rect &);

protected:
    Rect get_new_roi_rect(bool);

    void resize_Point(Rect, vector<Rect> &);

    Rect cal_the_rect_by_point(const vector<Point> &, int, int);

    bool judge_satisfy(const Mat &);

    bool judge_y_axis(const vector<Point> &, const vector<Point> &, int);

    bool judge_y_axis(Mat, const Rect &A_rect, const Rect &B_rect);

    bool judge_two_sides(const vector<Point> &, const vector<Point> &);

    vector<Rect> judge_contour(Mat src_image, const vector<Rect> &rect_vector);

    vector<Rect> sort_area(const vector<Rect> &);

    //select armour part
    vector<Rect> get_the_distance_vector(const vector<Rect> &);

    void push_back_none_vector(vector<vector<Rect> > &, int);

private:
    Mat collect_image;
    bool collect_flags;

    //VideoCapture index_cap;
    vector<struct combine_rect_height> contour_info;
};

#endif // !_ARMOUR_H
