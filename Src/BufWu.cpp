#include <iostream>
#include <string>
#include "BufWu.h"
#include<math.h>

Ligth::Ligth(string Train) {
    cout << "start" << Train << endl;
}

Ligth::~Ligth() {
}

void Ligth::display(string &Pic_Name, const Mat &Display_image) {
    imshow(Pic_Name, Display_image);
    waitKey(3);
}

Mat Ligth::Ligth_morpho_deal(Mat &mor_Pic) {
    Mat mor_out_Pic;
    Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
    morphologyEx(mor_Pic, mor_Pic, MORPH_OPEN, element);
    morphologyEx(mor_Pic, mor_out_Pic, MORPH_CLOSE, element);
    return mor_out_Pic;
}

Mat Ligth::Ligth_Pic_deal(Mat &Picture) {
    Mat put;
    int erosion_size = 7;
    Mat element = getStructuringElement(MORPH_RECT, Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                        Point(erosion_size, erosion_size));
    dilate(Picture, put, element);
    return put;
}

Mat Ligth::Ligth_Pic_Erode_deal(Mat &Picture) {
    Mat put;
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(Picture, put, element);
    return put;
}

Point2f Ligth::Point_Get(Point2f &Cente, Point2f &Start, double &angle) {
    Point Predict_Point;
    Predict_Point.x = (double) (Start.x - Cente.x) * (double) cos(angle * PI / 180) -
                      (double) (Start.y - Cente.y) * (double) sin(angle * PI / 180) + Cente.x;
    Predict_Point.y = (double) (Start.y - Cente.y) * (double) cos(angle * PI / 180) +
                      (double) (Start.x - Cente.x) * (double) sin(angle * PI / 180) + Cente.y;
    return Predict_Point;
}

void Ligth::Ligth_HSVimage_deal(Mat &tmp, Mat &result) {
    cvtColor(tmp, tmp, COLOR_BGR2YCrCb);
    vector<Mat> channels;
    split(tmp, channels);
    Mat Y = channels.at(0);
    Mat Cr = channels.at(1);
    Mat Cb = channels.at(2);
    result.create(tmp.rows, tmp.cols, CV_8UC1);
    for (int j = 1; j < Y.rows - 1; j++) {
        uchar *currentY = Y.ptr<uchar>(j);
        uchar *currentCr = Cr.ptr<uchar>(j);
        uchar *currentCb = Cb.ptr<uchar>(j);
        uchar *current = result.ptr<uchar>(j);
        for (int i = 1; i < Y.cols - 1; i++) {
            if ((currentCr[i] > 140) && (currentCr[i] < 255) && (currentY[i] > 0) && (currentY[i] < 255))
                current[i] = 0;
            else
                current[i] = 255;
        }
    }
    result = Ligth_Pic_Erode_deal(result);
    floodFill(result, Point(0, 0), Scalar(255));
    floodFill(result, Point(0, 0), Scalar(0));
    Y.release();
    Cr.release();
    Cb.release();
}

void Ligth::Circle_Center(Point2f POINT1, Point2f POINT2, double &R, Point2f &P_Center) {
    if (POINT1.x <= POINT2.x) {
        const double c1 = (POINT2.x * POINT2.x - POINT1.x * POINT1.x + POINT2.y * POINT2.y - POINT1.y * POINT1.y) /
                          (2 * (POINT2.x - POINT1.x));
        const double c2 = (POINT2.y - POINT1.y) / (POINT2.x - POINT1.x);
        const double A = (c2 * c2 + 1);
        const double B = (2 * POINT1.x * c2 - 2 * c1 * c2 - 2 * POINT1.y);
        const double C = POINT1.x * POINT1.x - 2 * POINT1.x * c1 + c1 * c1 + POINT1.y * POINT1.y - (R) * (R);
        P_Center.y = abs(-B - sqrt(B * B - 4 * A * C)) / (2 * A);
        P_Center.x = (c1 - c2 * P_Center.y);
    } else {
        const double c1 = (POINT2.x * POINT2.x - POINT1.x * POINT1.x + POINT2.y * POINT2.y - POINT1.y * POINT1.y) /
                          (2 * (POINT2.x - POINT1.x));
        const double c2 = (POINT2.y - POINT1.y) / (POINT2.x - POINT1.x);
        const double A = (c2 * c2 + 1);
        const double B = (2 * POINT1.x * c2 - 2 * c1 * c2 - 2 * POINT1.y);
        const double C = POINT1.x * POINT1.x - 2 * POINT1.x * c1 + c1 * c1 + POINT1.y * POINT1.y - (R) * (R);
        P_Center.y = abs(-B + sqrt(B * B - 4 * A * C)) / (2 * A);
        P_Center.x = (c1 - c2 * P_Center.y);
    }
}

void Ligth::findCircle2(Point2f &pt1, Point2f &pt2, Point2f &pt3, Point2f &CenTer1, double &R1) {
    double A1, A2, B1, B2, C1, C2, temp;
    A1 = pt1.x - pt2.x;
    B1 = pt1.y - pt2.y;
    C1 = (pow(pt1.x, 2) - pow(pt2.x, 2) + pow(pt1.y, 2) - pow(pt2.y, 2)) / 2.0;
    A2 = pt3.x - pt2.x;
    B2 = pt3.y - pt2.y;
    C2 = (pow(pt3.x, 2) - pow(pt2.x, 2) + pow(pt3.y, 2) - pow(pt2.y, 2)) / 2.0;
    temp = abs(A1 * B2 - A2 * B1);
    if (temp == 0) {
        CenTer1.x = 0;
        CenTer1.y = 0;
    } else {
        CenTer1.x = abs(C1 * B2 - C2 * B1) / temp;
        CenTer1.y = abs(A1 * C2 - A2 * C1) / temp;
    }
    R1 = sqrtf((CenTer1.x - pt1.x) * (CenTer1.x - pt1.x) + (CenTer1.y - pt1.y) * (CenTer1.y - pt1.y));
}

double Ligth::Point_Distance(Point2f &Point_F, Point2f &Point_S) {
    double Redious;
    Redious = sqrtf(
            (Point_F.x - Point_S.x) * (Point_F.x - Point_S.x) + (Point_F.y - Point_S.y) * (Point_F.y - Point_S.y));
    return Redious;
}

void Ligth::Dafu_Out(Mat &image, bool justy, Rect &roi, double &M_N, Point2f &Get_Point) {
    const static double angle = 10;
    Mat image4;
    Point2f Center_Point;
    Ligth_HSVimage_deal(image, image4);
    image4 = Ligth_Pic_Erode_deal(image4);
    image4 = Ligth_Pic_deal(image4);
    register int weigth, higth;
    register float Endx, Endy;
    vector<vector<Point> > contours0;
    vector<Vec4i> hierarchy0;
    findContours(image4, contours0, hierarchy0, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());
    cout << contours0.size() << endl;
    for (int first = 0; first < contours0.size(); first++) {
        RotatedRect rect_i = minAreaRect(contours0[first]);
        Point2f P_i[4];
        rect_i.points(P_i);
        Rect r_i = rect_i.boundingRect();
        register int lable = 0;
        if (r_i.height <= r_i.width) {
            higth = r_i.width;
            weigth = r_i.height;
        } else {
            higth = r_i.height;
            weigth = r_i.width;
        }

        double angle = (double) (higth / weigth);
        if (angle == 1 && contourArea(contours0[first]) >= 1000 && contourArea(contours0[first]) < 2500) {
            P_C[math] = rect_i.center;

            //P_C[math] = rect_i.center;
            if (math == 8) {

                findCircle2(P_C[math], P_C[math - 4], P_C[math - 8], Get_Center, R);
            }
            if (math >= 3) {
                Circle_Center(P_C[math], P_C[math - 2], R, P_Center[math]);
                if (Point_Distance(P_Center[math], P_Center[math - 1]) <= 5) {
                    circle(image, Point(P_Center[math].x, P_Center[math].y), 5, Scalar(0, 255, 0), -1, 8);
                }
            }

        }
        if (angle == 1 && P_Center[math].x != 0 && P_Center[math].y != 0 && contourArea(contours0[first]) >= 1000 &&
            contourArea(contours0[first]) < 2500) {

            for (int Rect_num = 0; Rect_num < 4; Rect_num++) {
                line(image, P_i[Rect_num], P_i[(Rect_num + 1) % 4], Scalar(0, 0, 255), 2, 8);
                circle(image, Point(rect_i.center.x, rect_i.center.y), 5, Scalar(0, 255, 0), -1, 8);
            }
            Center_Point = Point_Get(P_Center[math], rect_i.center, angle);
            roi = rect_i.boundingRect();
            circle(image, Point(Center_Point.x, Center_Point.y), 5, Scalar(0, 255, 0), -1, 8);
        }
        if (justy == false) {
            Get_Point = rect_i.center;
        } else if (Center_Point.x != 0 && Center_Point.y != 0) {
            Get_Point = Center_Point;
        }
        if (rect_i.center.x == 0 && rect_i.center.y == 0) {
            Get_Point = ERROR_POINT1;

        }
        if (math >= 800) {
            math = 0;
        }

        math++;
    }

}
