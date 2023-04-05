#include "armour.h"
// serialPosData
/*
 * this code was backup in 2019.07.06
 * author: Mr.Monster
 */

#define src_data_dir "/home/ubuntu/data/src_data/"
#define dst_data_dir "/home/ubuntu/data/dst_data/"
#define gray_data_dir "/home/ubuntu/data/gray_data/"
#define video_path "/home/nuc/video/"
//Mat collect_image;
//int collect_flags  = 0;
//int thread_flags = 0;
static int distance_times = 0;
int focus_global = 7.0;
int real_distance_height = 60;
static bool first_select_flags = false;
static double distance_single = 0;
double im_real_weights = 0;
double im_real_weights_buf = 5.5;
double angle_x_bias = 0;
double limit_angle_val = 2.0;
double angle_y_bias = 0;
bool target_flags = false;
extern int MOD_B_R;
bool thread_lauch_first = true;
bool process_finish = true;
//#define  thread_lauch 1
sem_t finish_sem;
sem_t first_sem;
//MOD_B_R=3;

int Polyfit_Get(double distance) {
    int Get_Ployfit =
            69.42 * pow(distance, 3) - 6.453 * pow(distance, 4) - 248.4 * pow(distance, 2) + 273 * distance + 192;
    return Get_Ployfit;
}


int get_usb_camara_serial(bool big_or_small = true, string path = "/sys/devices/pci0000:00/0000:00:14.0/usb1/") {
    cout << "path is :" << path << endl;
    //  vector<string> file_names;
    DIR *path_video;
    struct dirent *ptr;
    if (big_or_small == true) {
        cout << "big function" << endl;
        path += "1-4/1-4:1.0/video4linux/";
        if ((path_video = opendir((path + "video1/").c_str())) != NULL)// can't reach this path or can't get any file
        {
            return 1;
        } else if ((path_video = opendir((path + "video0/").c_str())) != NULL) {
            return 0;
        } else
            return -1;

    } else if (big_or_small == false) {
        cout << "small function " << endl;
        path += "1-2/1-2:1.0/video4linux/";
        if ((path_video = opendir((path + "video1/").c_str())) != NULL)// can't reach this path or can't get any file
        {
            return 1;
        } else if ((path_video = opendir((path + "video0/").c_str())) != NULL) {
            return 0;
        } else
            return -1;

    }
}


double pnp_Get_Distance_armour(Rect &roi) {
    double realistic_distance;
    Mat rotationMatrix, tvec;;

    //TODO 相机内参
    const static Mat cameraMatrix = (Mat_<double>(3, 3) << 1704.34015038228, 0.5471231529311396, 1338.01265844131, 0, 1704.21721269650, 788.218058016293, 0, 0, 1);
    //Matlab 转置的矩阵
    // 1704.34015038228 , 0 ,0
    // 0.5471231529311396, 1704.21721269650 ,0
    // 1338.01265844131, 788.218058016293, 1
    const static Mat distCoeffs = (Mat_<double>(1, 5) << -0.0082, -0.0392, -0.000880161386593770 ,-0.0013 ,0.0190);
    // 0.0082, -0.0392, -0.000880161386593770 ,-0.0013 ,0.0190

    //TODO 3D坐标
    vector<Point3f> realistic;
    realistic.push_back(cv::Point3f(0, 0, 0));
    realistic.push_back(cv::Point3f(0, 0.140, 0));
    realistic.push_back(cv::Point3f(0.06, 0.140, 0));
    realistic.push_back(cv::Point3f(0.06, 0, 0));
    vector<Point2f> point_get;
    point_get.push_back(Point2f(roi.x, roi.y));
    point_get.push_back(Point2f(roi.x + roi.width, roi.y));
    point_get.push_back(Point2f(roi.x + roi.width, roi.y + roi.height));
    point_get.push_back(Point2f(roi.x, roi.y + roi.height));
    Mat rvec(3, 1, DataType<double>::type);
    solvePnP(realistic, point_get, cameraMatrix, distCoeffs, rvec, tvec);
    Rodrigues(rvec, rotationMatrix);
    /// 2.0
    realistic_distance = tvec.at<double>(2, 0) * 2.0;
    im_real_weights = (double) real_distance_height / roi.height;
//    cout << "im_real_weights : " << im_real_weights << endl;
//    cout << "pnp distance is:" << realistic_distance << endl;
    return realistic_distance;
}

double pnp_Get_Distance_buf(Rect &roi) {
    double realistic_distance;
    Mat rotationMatrix, tvec;;
    const static Mat cameraMatrix = (Mat_<double>(3, 3) << 1704.34015038228, 0.5471231529311396, 1338.01265844131, 0, 1704.21721269650, 788.218058016293, 0, 0, 1);
    //Matlab 转置的矩阵
    // 1704.34015038228 , 0 ,0
    // 0.5471231529311396, 1704.21721269650 ,0
    // 1338.01265844131, 788.218058016293, 1
    const static Mat distCoeffs = (Mat_<double>(1, 5) << -0.0082, -0.0392, -0.000880161386593770 ,-0.0013 ,0.0190);
    // 0.0082, -0.0392, -0.000880161386593770 ,-0.0013 ,0.0190
    vector<Point3f> realistic;
    realistic.push_back(cv::Point3f(0, 0, 0));
    realistic.push_back(cv::Point3f(0, 0.3, 0));
    realistic.push_back(cv::Point3f(0.2, 0.3, 0));
    realistic.push_back(cv::Point3f(0.2, 0, 0));
    vector<Point2f> point_get;
    point_get.push_back(Point2f(roi.x, roi.y));
    point_get.push_back(Point2f(roi.x + roi.width, roi.y));
    point_get.push_back(Point2f(roi.x + roi.width, roi.y + roi.height));
    point_get.push_back(Point2f(roi.x, roi.y + roi.height));
    Mat rvec(3, 1, DataType<double>::type);
    solvePnP(realistic, point_get, cameraMatrix, distCoeffs, rvec, tvec);
    Rodrigues(rvec, rotationMatrix);

    realistic_distance = tvec.at<double>(2, 0) / 2.0;
    cout << "distance is:" << realistic_distance << endl;
    return realistic_distance;
}


static void *led_on_thread(void *arg) {
    int file_describe = gpio158_open();
    while (true) {
        switch (CMD_COLOR) {
            case 1:
                led_red(file_describe);
                break;
            case 2:
                led_blue(file_describe);
                break;
            default:
                gpio158_write_off(file_describe);
                break;
        }
    }
    gpio158_close(file_describe);
}


double armour::CalculateZ(float Digital_First_X, float Digital_Second_X, float Real_Distance, float focus) {
    float Image_W = Digital_Second_X - Digital_First_X;
    im_real_weights = (double) real_distance_height / Image_W;
    double Disitance = focus * Real_Distance / Image_W;
    return Disitance;
}


double get_sys_time() {
    return getTickCount();
}

void display(string window_name, const Mat &src_image) {
    imshow(window_name, src_image);
    waitKey(3);
}


//! 计算角度
//! \param prev_point
//! \param current_point
//! \param focus
//! \return
vector<double> armour::Calculate_angle(const Point &prev_point, const Point &current_point, double focus) {
    static int run_time_count = 0;//check whether the car is moving
    double x_bias = current_point.x - prev_point.x;
    double y_bias = current_point.y - prev_point.y;
    vector<double> angle_vector;
    angle_vector.push_back(atan(x_bias * im_real_weights / 100 / focus) * 180 / 3.1415926);
    angle_vector.push_back(atan(y_bias * im_real_weights / 100 / focus) * 180 / 3.1415926);
    cout << "im_real_weights:" << im_real_weights << endl;
    cout << "x_bias:" << x_bias << endl;
    cout << "y_bias:" << y_bias << endl;
    cout << "focus :" << focus << endl;
    cout << " angle Percent: " << x_bias * im_real_weights / 100 / focus << endl;

    //传出数据
    data_send.x = x_bias ;
    data_send.y = y_bias ;

    //cout << "angle check : "<< angle_vector[0] << endl;
    if (angle_vector[0] < 0 && fabs(angle_vector[0]) > limit_angle_val) {
        angle_vector[0] -= angle_x_bias;
    } else if (angle_vector[0] > 0 && fabs(angle_vector[0]) > limit_angle_val) {
        angle_vector[0] += angle_x_bias;
    }
    if (angle_vector[1] < 0 && fabs(angle_vector[1]) > limit_angle_val) {
        angle_vector[1] -= angle_y_bias;
    } else if (angle_vector[1] > 0 && fabs(angle_vector[1]) > limit_angle_val) {
        angle_vector[1] += angle_y_bias;
    }

    return angle_vector;
}


vector<double> Calculate_angle_buf(const Point &prev_point, const Point &current_point, double focus) {

    static int run_time_count = 0;//check whether the car is moving
    double x_bias = current_point.x - prev_point.x;
    double y_bias = current_point.y - prev_point.y;
    vector<double> angle_vector;
    angle_vector.push_back(atan(x_bias * im_real_weights_buf / 100 / focus) * 180 / 3.1415926);
    angle_vector.push_back(atan(y_bias * im_real_weights_buf / 100 / focus) * 180 / 3.1415926);
    cout << "angle check : " << angle_vector[0] << endl;
    return angle_vector;
}


armour::armour(int index) {
    //VideoCapture tmp_cap(index);
    //index_cap = tmp_cap;
}

armour::~armour() {

}


void armour::Port_armour() {

}

void mytrackBarCallback(int value, void *data) {
    *((int *) data) = value;
    cout << "value : " << value << endl;
}


Mat HSV_to_gray(const Mat &src_image, int &h_min, int &h_max, int &s_min, int &s_max, int &v_min, int &v_max) {
    Mat hsv_image;
    cvtColor(src_image, hsv_image, COLOR_BGR2HSV);

    Mat dst_image;
    //namedWindow("image show", WINDOW_FREERATIO);
    inRange(hsv_image, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), dst_image);

    //createTrackbar("h_min", "image show", &val, 255, mytrackBarCallback, &h_min);
    //imshow("image show",dst_image);
    return dst_image;

}

Mat armour::HSV_to_gray(const Mat &src_image, int mode, int h_min = 0, int h_max = 255, int s_min = 0, int s_max = 255,
                        int v_min = 0, int v_max = 255) {
    Mat hsv_image;
    cvtColor(src_image, hsv_image, COLOR_BGR2HSV);
    int val = 0;
    switch (mode) {
        case MODE_BLUE:
            h_min = 45;
            h_max = 205;
            s_min = 132;
            s_max = 255;
            v_min = 136;
            v_max = 255;
            break;
        case MODE_RED:
            h_min = 140;
            h_max = 189;
            s_min = 160;
            s_max = 255;
            v_min = 30;
            v_max = 223;
            break;
        case MODE_NONE:
            break;
        default :
            break;
    }
    if (mode == MODE_NONE)
        cout << "argument wrong !" << endl;
    Mat dst_image;

    //namedWindow("image show", WINDOW_FREERATIO);

    inRange(hsv_image, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), dst_image);

    //createTrackbar("h_min", "image show", &val, 255, mytrackBarCallback, &h_min);
    //imshow("image show",dst_image);
    return dst_image;

}


Mat armour::YUV_to_gray(const Mat &src_image_param, int Mode, int Cb_min = 0, int Cb_max = 255, int Cr_min = 0,
                        int Cr_max = 255, int Y_min = 0, int Y_max = 255) {
    Mat src_image;
    cvtColor(src_image_param, src_image, COLOR_BGR2YCrCb);
    vector<Mat> src_channels;
    split(src_image, src_channels);
    Mat image_Cb = src_channels.at(2);
    Mat image_Cr = src_channels.at(1);
    Mat image_Y = src_channels.at(0);
    Mat result_pic(src_image.rows, src_image.cols, CV_8UC1);
    switch (Mode) {
        case MODE_BLUE:
            Cb_min = 129;
            Cb_max = 255;
            Y_min = 224;
            Y_max = 250;
            Cr_min = 99;
            Cr_max = 122;
            break;
        case MODE_RED:
            Cb_min = 57;
            Cb_max = 189;
            Cr_min = 152;
            Cr_max = 255;
            Y_min = 204;
            Y_max = 239;
            break;
        default:
            break;
    }
    if (Mode == MODE_NONE && Cb_min == 0 && Cb_max == 255 && Y_max == 255 && Y_min == 0 && Cr_min == 0 &&
        Cr_max == 255) {
        cout << "input error,you need put the argument" << endl;
        //exit(1);
    }
    for (int rows = 0; rows < src_image.rows; rows++) {
        uchar *image_Cb_rows = image_Cb.ptr<uchar>(rows);
        uchar *image_Y_rows = image_Y.ptr<uchar>(rows);
        uchar *image_Cr_rows = image_Cr.ptr<uchar>(rows);
        uchar *image_result_rows = result_pic.ptr<uchar>(rows);
        //uchar * image_result_rows = result_pic.ptr<uchar>(rows);
        for (int cols = 0; cols < src_image.cols; cols++) {
            if (image_Cb_rows[cols] >= Cb_min && image_Cb_rows[cols] <= Cb_max && image_Y_rows[cols] >= Y_min &&
                image_Y_rows[cols] <= Y_max && image_Cr_rows[cols] >= Cr_min && image_Cr_rows[cols] <= Cr_max) {
                image_result_rows[cols] = 255;
            } else {
                image_result_rows[cols] = 0;
            }
        }

    }
    return result_pic;
}


Mat armour::rgb_to_gray(const Mat &src_image, int Mode, int R_min = 0, int R_max = 255, int G_min = 0, int G_max = 255,
                        int B_min = 0, int B_max = 255) {

    vector<Mat> src_channels;
    split(src_image, src_channels);
    Mat image_red = src_channels.at(2);
    Mat image_green = src_channels.at(1);
    Mat image_blue = src_channels.at(0);
    Mat result_pic(src_image.rows, src_image.cols, CV_8UC1);
    switch (Mode) {
        case MODE_BLUE:
            R_min = 92;
            R_max = 255;
            B_min = 159;
            B_max = 217;
            G_min = 140;
            G_max = 255;
            break;
        case MODE_RED:
            R_min = 255;
            R_max = 255;
            G_min = 0;
            G_max = 255;
            B_min = 0;
            B_max = 255;
            break;
        default:
            break;
    }
    if (Mode == MODE_NONE && R_min == 0 && R_max == 255 && B_max == 255 && B_min == 0 && G_min == 0 && G_max == 255) {
        cout << "input error,you need put the argument" << endl;
        //exit(1);
    }
    for (int rows = 0; rows < src_image.rows; rows++) {
        uchar *image_red_rows = image_red.ptr<uchar>(rows);
        uchar *image_blue_rows = image_blue.ptr<uchar>(rows);
        uchar *image_green_rows = image_green.ptr<uchar>(rows);
        uchar *image_result_rows = result_pic.ptr<uchar>(rows);
        for (int cols = 0; cols < src_image.cols; cols++) {
            if (image_red_rows[cols] >= R_min && image_red_rows[cols] <= R_max && image_blue_rows[cols] >= B_min &&
                image_blue_rows[cols] <= B_max && image_green_rows[cols] >= G_min && image_green_rows[cols] <= G_max) {
                image_result_rows[cols] = 255;
            } else {
                image_result_rows[cols] = 0;
            }
        }

    }
    return result_pic;
}

//TODO 判断颜色
auto armour::judgeColor(const Mat &src_image) {
    // 转换为Lab颜色空间
    Mat lab;
    cvtColor(src_image, lab, COLOR_BGR2Lab);

    // 分离L、a、b通道
    vector<cv::Mat> channels;
    split(lab, channels);

    // 计算直方图
    int histSize[] = {256};
    float lRanges[] = {0, 256};
    const float *ranges[] = {lRanges};
    cv::MatND hist;
    cv::calcHist(&channels[0], 1, 0, cv::Mat(), hist, 1, histSize, ranges, true, false);

    // 找到最大值
    double maxVal = 0;
    int maxIdx = 0;
    for (int i = 0; i < histSize[0]; i++) {
        double val = hist.at<float>(i);
        if (val > maxVal) {
            maxVal = val;
            maxIdx = i;
        }
    }

    // 判断颜色
    if (maxIdx >= 165 && maxIdx <= 255) {
        return RED;
    } else if (maxIdx >= 0 && maxIdx <= 95) {
        return BLUE;
    } else {
        return NONE;
    }
}

Mat armour::erode_pic(const Mat &src_image, int size_element = 1) {
    Mat dst_image(src_image.rows, src_image.cols, CV_8UC1);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(size_element, size_element));
    erode(src_image, dst_image, kernel);
    return dst_image;
}


Mat armour::dilate_pic(const Mat &src_image, int size_element = 3) {
    Mat dst_image(src_image.rows, src_image.cols, CV_8UC1);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(size_element, size_element));
    dilate(src_image, dst_image, kernel);
    return dst_image;
}

vector<vector<Point>> armour::get_point_contours(const Mat &src_image)//3
{
    vector<vector<Point>> contour_vector;
    vector<Vec4i> contour_hierachy;
//    cout << "hello1 " << endl;
    findContours(src_image, contour_vector, contour_hierachy, RETR_CCOMP, CHAIN_APPROX_TC89_L1);
//    cout << "hello2" << endl;
    //cout << "contour_vector size:" << contour_vector.size() << endl;
    return contour_vector;
}


bool armour::judge_satisfy(const Mat &Roi) {
    cout << "size: height:" << Roi.rows << " width: " << Roi.cols << endl;
    cout << "percent : " << (double) Roi.rows / Roi.cols << endl;
    //waitKey();

    if ((double) Roi.rows / Roi.cols < 1.40 || (double) Roi.rows / Roi.cols > 4.00)
        return false;
    /*if((double) Roi.rows / Roi.cols < 1.0)
        return false;
    /*if (Roi.rows > 200 || Roi.cols > 200)
        return false;*/
    /*imshow("roi", Roi);
    waitKey();*/
    return true;
}

/*
 * 作者:Monster
 * 函数名称:get_rect_pic_contour
 * 传入参数:二值化后的图片,轮廓描绘的点的容器,用于保存合格的轮廓的容器
 * 传出变量:空
 * 函数作用:用于剔除轮廓中不合格的
 * 说明:无 
 * */


void armour::get_rect_pic_contour(Mat gray_image, const vector<vector<Point>> &contour_vector,
                                  vector<vector<Point>> &contour_rect)//4
{
    //cout << "enter get_rect_pic_contour func" << endl;
    for (int num = 0; num < contour_vector.size(); num++) {
        Rect tmp_contour = boundingRect(contour_vector[num]);
        Mat ROI = gray_image(tmp_contour);
        //display("ROI", ROI);
        if (judge_satisfy(ROI) == false) {
            continue;
        }
        contour_rect.push_back(contour_vector[num]);
    }
}

/*
 * 作者:Monster
 * 函数名称:get_rotate_angle
 * 传入参数:findcontours传出的轮廓角点容器
 * 传出变量:各个轮廓的角度
 * 函数作用:计算各个轮廓的角度
 * 说明:
 * */

vector<double> armour::get_rotate_angle(const vector<vector<Point>> &rotate_rect) {
    vector<double> theata_vector;
    for (int num = 0; num < rotate_rect.size(); num++) {
        int flags = 0;
        if (rotate_rect[num][0].x < rotate_rect[num][0].x) flags = 1;
        double gradient = fabs((double) (rotate_rect[num][1].y - rotate_rect[num][0].y)) /
                          fabs((double) (rotate_rect[num][1].x - rotate_rect[num][0].x));
        double theata = atan(gradient) * 180.0 / 3.1415926;
        if (flags == 1) theata = 180 - theata;
        theata_vector.push_back(theata);
    }
    return theata_vector;
}

vector<double> armour::get_rotate_angle(Mat &src_image, const vector<vector<Point>> &rotate_rect) {
    vector<double> theata_vector;
    for (int num = 0; num < rotate_rect.size(); num++) {
        int flags = 0;
        /*if (rotate_rect[num][0].x < rotate_rect[num][0].x) flags = 1;
        double gradient = fabs((double)(rotate_rect[num][1].y - rotate_rect[num][0].y)) / fabs((double)(rotate_rect[num][1].x - rotate_rect[num][0].x));
        //if(num == 0)
        {
            //cout << "size:" << rotate_rect[num].size() << endl;
            circle(src_image,rotate_rect[num][0],1,Scalar(0,0,255));
            circle(src_image,rotate_rect[num][1],1,Scalar(0,255,0));
            circle(src_image,rotate_rect[num][2],1,Scalar(255,255,0));
            circle(src_image,rotate_rect[num][4],1,Scalar(0,255,255));
        }*/
        Rect tmp_rect = boundingRect(rotate_rect[num]);
        Point left_up(src_image.cols, tmp_rect.y);
        Point left_down(src_image.cols, tmp_rect.y + tmp_rect.height);
        for (int obj_num = 0; obj_num < rotate_rect[num].size(); obj_num++) {
            if (rotate_rect[num][obj_num].y == tmp_rect.y)
                if (rotate_rect[num][obj_num].x < left_up.x)
                    left_up.x = rotate_rect[num][obj_num].x;
            if (rotate_rect[num][obj_num].y == tmp_rect.y + tmp_rect.height - 1)
                if (rotate_rect[num][obj_num].x < left_down.x)
                    left_down.x = rotate_rect[num][obj_num].x;
        }
        if (left_down.x > left_up.x) flags = 1;
        //circle(src_image,left_up,1,Scalar(0,0,255),2,8);
        //circle(src_image,left_down,1,Scalar(0,255,255),2,8);
        double gradient = fabs((double) (left_down.y - left_up.y)) / fabs((double) (left_down.x - left_up.x));
        double theata = atan(gradient) * 180.0 / 3.1415926;
        if (flags == 1) theata = 180 - theata;
        //cout << "angle: " << theata << endl;
        theata_vector.push_back(theata);
    }
    return theata_vector;
}

/*void armour::get_rotate_angle(const vector<vector<Point> > & rotate_rect, vector<double>& rotate_angle)
{
	cout << "enter get_rotate_angle func" << endl;
	cout << "rotate_rect size:" << rotate_rect.size() << endl;
	for (int num = 0; num < rotate_rect.size(); num++)
	{
		RotatedRect tmp_rotate_rect(rotate_rect[num][0], rotate_rect[num][1], rotate_rect[num][2]);
		cout << "angle is :" << tmp_rotate_rect.angle << endl;
		rotate_angle.push_back(tmp_rotate_rect.angle;
	}
}*/


/*
 * 作者:Monster
 * 传入参数:所获得的各个轮廓的角度
 * 传出变量:各个函数的角度的差值和各个轮廓在容器中的位置
 * 函数作用:算出两两轮廓之间角度的变化值
 * 说明:Scalar中第一个val中的第一个变量的值是两个对比的轮廓的角度
 * 		第二个变量是对比的轮廓在轮廓容器中的排列位置
 * 		第三个变量是对比的第二个轮廓在容器中的排列位
 * 		该函数中不存在筛选函数调用
 * */

vector<Scalar> armour::cal_distance_theata(const vector<double> &theata_vector) {
    //cout << "enter cal_distance_theata function" << endl;
    vector<Scalar> tmp_Scalar_vector;
    if (theata_vector.size() == 0 || theata_vector.size() == 1)//如果发现角度容器中存放的只有一个轮廓的角度,那么这时候返回一空容器
    {
        return tmp_Scalar_vector;
    }
    for (int out_side = 0; out_side < theata_vector.size() - 1; out_side++)//便利整个容器进行一一对比
    {
        for (int index_side = out_side + 1; index_side < theata_vector.size(); index_side++) {
            double distance_theata = fabs(theata_vector[out_side] - theata_vector[index_side]);//两者角度差值的绝对值定义为两个轮廓之间的距离
            Scalar tmp_scalar(distance_theata, out_side, index_side);
            tmp_Scalar_vector.push_back(tmp_scalar);
        }
    }
    return tmp_Scalar_vector;
}


/*
 * 作者:Monster
 * 传入参数:获得的各个轮廓的角度差 最大允许范围角度
 * 传出变量:经过筛选后的轮廓的角度差
 * 函数作用:筛选,将角度差大于指定值的角度剔除
 * 说明:Scalar中第一个val中的第一个变量的值是两个对比的轮廓的角度
 * 		第二个变量是对比的轮廓在轮廓容器中的排列位置
 * 		第三个变量是对比的第二个轮廓在容器中的排列位
 * 		该函数中不存在筛选函数调用
 * */

vector<Scalar> armour::judge_the_theata(const vector<Scalar> &angle_Scalar, double max_angle = 20) {
    //cout << "enter judge the theata func" << endl;
    //cout << "angle_Scalar size is: " << angle_Scalar.size() << endl;
    vector<Scalar> tmp_vector_angle;
    for (int num = 0; num < angle_Scalar.size(); num++) {
        cout << "angle is : " << angle_Scalar[num].val[0] << endl;
        if (angle_Scalar[num].val[0] > max_angle) continue;
        tmp_vector_angle.push_back(angle_Scalar[num]);
    }
    return tmp_vector_angle;
}

/*
 * 作者:Monster
 * 函数名称:set_the_rect
 * 传入参数:二值图 含有不同轮廓角度差的容器 灰度图中所找到轮廓的点的容器 预测到的可能装甲板的容器 图片的高度 图片的宽度
 * 传出变量:空
 * 函数作用:根据筛选到的配对好的轮廓的对数来找到对应的正方形
 * 说明:
 * */

void armour::set_the_rect(const Mat &gray_image, const vector<Scalar> &place_info,
                          const vector<vector<Point>> &contour_point_vector, vector<Rect> &armour_place_info,
                          int pic_max_rows = 480, int pic_max_cols = 640) {
    //double debug_time = get_sys_time();
    for (register int num = 0; num < place_info.size(); num++) {
        vector<Point> tmp_armour;
        register int out_side = (int) place_info[num].val[1];
        register int in_side = (int) place_info[num].val[2];//
        /*Rect tmp_a_rect, tmp_b_rect;
        tmp_a_rect = boundingRect(contour_point_vector[out_side]);
        tmp_b_rect = boundingRect(contour_point_vector[in_side]);
        Mat a_image = gray_image(tmp_a_rect);
        Mat b_image = gray_image(tmp_b_rect);
        imshow("a", a_image);
        imshow("b", b_image);*/
        if (!judge_two_sides(contour_point_vector[out_side], contour_point_vector[in_side])) continue;
        //if (!judge_y_axis(contour_point_vector[out_side],contour_point_vector[in_side],gray_image.rows)) continue;
        if (!judge_y_axis(gray_image, boundingRect(contour_point_vector[out_side]),
                          boundingRect(contour_point_vector[in_side])))
            continue;
        for (register int out_num = 0; out_num < contour_point_vector[out_side].size(); out_num++) {
            tmp_armour.push_back(contour_point_vector[out_side][out_num]);
        }
        for (register int in_num = 0; in_num < contour_point_vector[in_side].size(); in_num++) {
            tmp_armour.push_back(contour_point_vector[in_side][in_num]);
        }
        armour_place_info.push_back(cal_the_rect_by_point(tmp_armour, pic_max_rows, pic_max_cols));
        //display("debug", gray_image(cal_the_rect_by_point(tmp_armour, pic_max_rows, pic_max_cols)));
        //waitKey();

    }
    //debug_time = get_sys_time() - debug_time;
    //cout << "debug_time : " << debug_time / getTickFrequency() << endl;
}

bool
armour::judge_y_axis(const vector<Point> &point_A_vector, const vector<Point> &point_B_vector, int size_y = 480) {
    register int A_y_min = size_y;
    register int A_y_max = 0;
    register int B_y_min = size_y;
    register int B_y_max = 0;
    for (int obj_num = 0; obj_num < point_A_vector.size(); obj_num++) {
        if (point_A_vector[obj_num].y > A_y_max)
            A_y_max = point_A_vector[obj_num].y;
        if (point_A_vector[obj_num].y < A_y_min)
            A_y_min = point_A_vector[obj_num].y;
    }
    for (int obj_num = 0; obj_num < point_B_vector.size(); obj_num++) {
        if (point_B_vector[obj_num].y > B_y_max)
            B_y_max = point_B_vector[obj_num].y;
        if (point_B_vector[obj_num].y < B_y_min)
            B_y_min = point_B_vector[obj_num].y;
    }
    /*if((A_y_max > B_y_min && A_y_max < B_y_max) || (B_y_max > A_y_min && B_y_max < A_y_max))
        return true;*/
    /*if(A_y_max < B_y_min || B_y_max < B_y_min)
        return false;*/
    cout << "A_y_max:" << A_y_max << " A_y_min:" << A_y_min << endl;
    cout << "B_y_max:" << B_y_max << " B_y_min:" << B_y_min << endl;
    //waitKey();
    return false;
}


bool armour::judge_y_axis(Mat src_image, const Rect &A_rect, const Rect &B_rect) {
    Mat dst_image = src_image.clone();
    int A_y_min = A_rect.y;
    int A_y_max = A_rect.y + A_rect.height;
    int B_y_min = B_rect.y;
    int B_y_max = B_rect.y + B_rect.height;
    rectangle(dst_image, A_rect, Scalar(255, 255, 255), 2, 8);
    rectangle(dst_image, B_rect, Scalar(255, 255, 255), 2, 8);
    //imshow("debug",dst_image);
    /*if((A_y_max > B_y_min && A_y_max < B_y_max) || (B_y_max > A_y_min && B_y_max < A_y_max))
        return true;
    if (fabs(B_y_min - A_y_max )< 50  && fabs(B_y_min - A_y_max) > 20 )
        return true;
    */
    //if(A_y_max > B_y_min && )
    /*if(fabs (A_y_max - B_y_max ) <50 && fabs(B_y_min - A_y_min) < 50)
        return true;*/
    double gradient = fabs(((double) A_rect.y - B_rect.y) / (A_rect.x - B_rect.x));
    double angle = atan(gradient) * 180.0 / 3.1415926;
    if (angle > 18) return false;
    cout << "theata : " << angle << endl;
    /*cout << "b_min - a_max:" << B_y_min - A_y_max << endl;
    cout << "A_y_max:" << A_y_max << " A_y_min:" << A_y_min << endl;
    cout << "B_y_max:" << B_y_max << " B_y_min:" << B_y_min << endl;*/
    //waitKey();
    return true;
}

/*
 * 作者:Monster
 * 函数名称:judge_two_sides
 * 传入参数:一对配对的轮廓所构成的点的容器A 一对配对的轮廓所构成的点的容器B
 * 传出变量:轮廓是否符合要求
 * 函数作用:通过判断一对配对的轮廓两端最长的距离来判断这两个轮廓是否符合要求
 * 说明:
 * */


bool armour::judge_two_sides(const vector<Point> &point_A_vector, const vector<Point> &point_B_vector) {
    register double a_max_distance = 0;
    register double b_max_distance = 0;
    register double tmp_distance;
    //cout << "size_A:" << point_A_vector.size() << "B:" << point_B_vector.size() << endl;
    //找到组成轮廓A的最长的距离
    for (register int obj_num = 0; obj_num < point_A_vector.size() - 1; obj_num++) {
        for (register int index_num = obj_num; index_num < point_A_vector.size(); index_num++) {
            tmp_distance = sqrt(pow((double) (point_A_vector[obj_num].x - point_A_vector[index_num].x), 2) +
                                pow((double) (point_A_vector[obj_num].y - point_A_vector[index_num].y), 2));
            if (tmp_distance > a_max_distance)
                a_max_distance = tmp_distance;
        }
    }
    //找到组成轮廓B的最长的距离
    for (register int obj_num = 0; obj_num < point_B_vector.size() - 1; obj_num++) {
        for (register int index_num = obj_num; index_num < point_B_vector.size(); index_num++) {
            tmp_distance = sqrt(pow((double) (point_B_vector[obj_num].x - point_B_vector[index_num].x), 2) +
                                pow((double) (point_B_vector[obj_num].y - point_B_vector[index_num].y), 2));
            if (tmp_distance > b_max_distance)
                b_max_distance = tmp_distance;
        }
    }
    /*if ((double)a_max_distance / b_max_distance < 0.9 || (double)a_max_distance / b_max_distance > 1.1 || (double)b_max_distance / a_max_distance > 1.1 || (double)b_max_distance / a_max_distance < 0.9)
    {

        return false;
    }*/

    if ((double) a_max_distance / b_max_distance < 0.69 || (double) a_max_distance / b_max_distance > 1.44 ||
        (double) b_max_distance / a_max_distance > 1.44 || (double) b_max_distance / a_max_distance < 0.69) {

        return false;
    }
    cout << "a_max_distance : " << a_max_distance << " b_max_distance:" << b_max_distance << endl;
    cout << "a/b:" << a_max_distance / b_max_distance << "b/a" << b_max_distance / a_max_distance << endl;
    cout << "|a-b| " << fabs(a_max_distance - b_max_distance) << endl;
    //waitKey();
    return true;
}

Rect armour::cal_the_rect_by_point(const vector<Point> &point_vector, int init_rows = 480, int init_cols = 640) {
    register int min_x = init_cols, min_y = init_rows;
    register int max_x = 0, max_y = init_rows = 0;

    for (register int obj_num = 0; obj_num < point_vector.size(); obj_num++) {
        if (min_x > point_vector[obj_num].x) min_x = point_vector[obj_num].x;
        if (min_y > point_vector[obj_num].y) min_y = point_vector[obj_num].y;
        if (max_x < point_vector[obj_num].x) max_x = point_vector[obj_num].x;
        if (max_y < point_vector[obj_num].y) max_y = point_vector[obj_num].y;
    }
    Rect tmp_rect(Point(min_x, min_y), Point(max_x, max_y));
    return tmp_rect;
}

bool judge_rect_satisfy(const Rect &Roi) {
    cout << "rows : " << Roi.height << " cols : " << Roi.width << endl;
    cout << "percent : " << (double) Roi.height / Roi.width << endl;
    cout << "area: " << Roi.height * Roi.width << endl;
    //waitKey();
    /*if (Roi.height > 140)
        return false;
    if ((double)Roi.height / Roi.width < 0.18||(double)Roi.height/Roi.width > 0.20 && (double)Roi.height/Roi.width < 0.38|| ((double)Roi.height/Roi.width) > 0.50)
        return false;
    if (Roi.height * Roi.width > 3050)
        return false;*/
    if ((double) Roi.height / Roi.width > 0.9)
        return false;
    if ((double) Roi.height / Roi.width < 0.25)
        return false;
    /*if((double)Roi.height / Roi.width > 0.34 && (double)Roi.height / Roi.width < 0.38)
        return false;*/
    /*if(Roi.width > 250 && ((double)Roi.height / Roi.width < 0.4 || (double ) Roi.height / Roi.width > 0.5 ))
        return false;*/
    //waitKey();
    /*if ((double)Roi.height / Roi.width < 0.25 || (double)Roi.height / Roi.width > 0.40)
        return false;*/
    return true;

}

vector<vector<Point>>
armour::judge_the_min_distance(const Mat &gray_image, const vector<vector<Point>> &vector_point, double max_value = 25,
                               double min_value = 4.1) {
    register double min_distance = DBL_MAX;
    register double current_distance;
    //cout << "min_distance:" << min_distance << endl;
    vector<vector<Point>> select_vector_point;
    for (register int obj_num = 0; obj_num < vector_point.size(); obj_num++) {
        /*for (register int rows = 0; rows < vector_point[obj_num].size() - 1; rows++)
            {
                for (register int cols = rows + 1; cols < vector_point[obj_num].size(); cols++)
                {
                    current_distance = sqrt(pow(((double)vector_point[obj_num][rows].x - vector_point[obj_num][cols].x), 2) + pow((double)vector_point[obj_num][rows].y - vector_point[obj_num][cols].y, 2));
                    //cout << "current_distance:"<< current_distance << endl;
                    if (min_distance > current_distance)
                    {
                        cout << "change" <<  min_distance<< endl;
                        min_distance = current_distance;
                    }
                }
            }*/
        current_distance = sqrt(pow(((double) vector_point[obj_num][3].x - vector_point[obj_num][0].x), 2) +
                                pow((double) vector_point[obj_num][3].y - vector_point[obj_num][0].y, 2));
        //cout << "debug:" << current_distance << endl;
        if (current_distance <= max_value && current_distance >= min_value)
            select_vector_point.push_back(vector_point[obj_num]);
        //cout << "min_distance:" << (double)min_distance << endl;
        //display("debug", gray_image(boundingRect(vector_point[obj_num])));
        //waitKey();
        min_distance = DBL_MAX;
        //cout << "change min_distance:" << min_distance<< endl;
    }
    return select_vector_point;
}

vector<Rect> armour::bubble_sort(vector<double> distance, vector<Rect> distance_Rect) {
    Rect tmp_Rect;
    register int tmp_index;
    if (!distance.size())
        return distance_Rect;
    for (register int obj_num = 0; obj_num < distance.size(); obj_num++) {
        for (register int index_num = 0; index_num < distance.size() - 1 - obj_num; index_num++) {
            if (distance[index_num + 1] < distance[index_num]) {
                tmp_index = distance[index_num + 1];
                distance[index_num + 1] = distance[index_num];
                distance[index_num] = tmp_index;
                tmp_Rect = distance_Rect[index_num + 1];
                distance_Rect[index_num + 1] = distance_Rect[index_num];
                distance_Rect[index_num] = tmp_Rect;
            }
        }
    }
    return distance_Rect;
}


Mat armour::rect_the_pic(const Mat &gray_image, Mat &src_image, vector<Rect> &rect_vector) {
    Mat dst_image;
    vector<Rect> rect_vector_tmp;
    src_image.copyTo(dst_image);
    rect_vector = judge_contour(gray_image, rect_vector);
    for (register int obj_num = 0; obj_num < rect_vector.size(); obj_num++) {
        Mat debug_pic = src_image(rect_vector[obj_num]);

        //display("debug", debug_pic);
        if (!judge_rect_satisfy(rect_vector[obj_num])) continue;
        rect_vector_tmp.push_back(rect_vector[obj_num]);
        //rectangle(dst_image, rect_vector[obj_num], Scalar(255, 255, 0));
    }
    rect_vector = rect_vector_tmp;

    return dst_image;
}

double armour::get_max_area(vector<Rect> rect_vector) {
    //Rect tmp_rect;
    register double max_area;
    if (!rect_vector.size())
        return 0;
    max_area = rect_vector[0].area();
    for (register int obj_num = 0; obj_num < rect_vector.size(); obj_num++) {
        if (max_area < rect_vector[obj_num].area()) {
            max_area = rect_vector[obj_num].area();
        }
    }
    return max_area;
}

Point armour::select_the_rect_area(Mat &src_image, const vector<Rect> &vector_rect) {
    register int index_obj = 0;
    register double max_area = 0;
    if (vector_rect.size() == 0)return ERROR_POINT;
    for (int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {
        if (vector_rect[obj_num].area() > max_area) {
            max_area = vector_rect[obj_num].area();
            index_obj = obj_num;
        }
    }
    Point armour_point = Point(vector_rect[index_obj].x + vector_rect[index_obj].width / 2.0,
                               vector_rect[index_obj].y + vector_rect[index_obj].height / 2.0);
    rectangle(src_image, vector_rect[index_obj], Scalar(0, 255, 255), 2, 8);
    return armour_point;
}

/*
 * 作者:Monster
 * 函数名称:sort_area
 * 传入参数:never sort vector of rect
 * 传出变量:vector of rect which has been sorted
 * 函数作用:sort the vector from the area
 * 说明:
 * */

vector<Rect> armour::sort_area(const vector<Rect> &rect_vector) {
    double max_area = 0;
    double min_area = DBL_MAX;
    double area_count = 0;
    vector<Rect> tmp_sorted_area;
    for (int obj_num = 0; obj_num < rect_vector.size(); obj_num++) {
        if (rect_vector[obj_num].area() > max_area)
            max_area = rect_vector[obj_num].area();
        if (rect_vector[obj_num].area() < min_area) min_area = rect_vector[obj_num].area();
    }
    double tmp_area_sum = min_area + max_area;
    double ave_area = tmp_area_sum / rect_vector.size();
    for (int obj_num = 0; obj_num < rect_vector.size(); obj_num++) {
        if (rect_vector[obj_num].area() < ave_area) continue;
        tmp_sorted_area.push_back(rect_vector[obj_num]);
    }
    return tmp_sorted_area;
}

void armour::push_back_none_vector(vector<vector<Rect>> &rect_vector, int size) {
    for (int obj_num = 0; obj_num < size; obj_num++) {
        vector<Rect> tmp_rect_vector;
        rect_vector.push_back(tmp_rect_vector);
    }
    return;
}

vector<Rect> armour::get_the_distance_vector(const vector<Rect> &vector_rect) {
    vector<double> distance_tmp;
    for (int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {
        distance_tmp.push_back(CalculateZ(0, vector_rect[obj_num].height, real_distance_height, focus_global));
    }
    vector<vector<Rect>> distance_vector;
    push_back_none_vector(distance_vector, 6);
    for (int obj_num = 0; obj_num < distance_tmp.size(); obj_num++) {
        if (distance_tmp[obj_num] < 12)
            distance_vector[0].push_back(vector_rect[obj_num]);
        else if (distance_tmp[obj_num] < 25)
            distance_vector[1].push_back(vector_rect[obj_num]);
        else if (distance_tmp[obj_num] < 35)
            distance_vector[2].push_back(vector_rect[obj_num]);
        else if (distance_tmp[obj_num] < 45)
            distance_vector[3].push_back(vector_rect[obj_num]);
        else if (distance_tmp[obj_num] < 55)
            distance_vector[4].push_back(vector_rect[obj_num]);
        else if (distance_tmp[obj_num] < 65)
            distance_vector[5].push_back(vector_rect[obj_num]);
    }
    for (int obj_num = 0; obj_num < distance_vector.size(); obj_num++) {
        if (distance_vector[obj_num].size() != 0) {
            cout << "select obj_num : " << obj_num << endl;
            return distance_vector[obj_num];
            //break;
        }
    }
    //vector<Rect> rect_tmp;
    return vector_rect;

}


/*vector<Rect> armour::get_the_distance_vector(const  vector<Rect> & vector_rect)
{
	vector<vector<Rect> > distance_vector;
	push_back_none_vector(distance_vector,6);
	for (int obj_num = 0; obj_num < vector_rect.size(); obj_num++)
	{
		if(vector_rect[obj_num].area() > 2000)
			distance_vector[0].push_back(vector_rect[obj_num]);
		else if(vector_rect[obj_num].area() > 2900)
			distance_vector[1].push_back(vector_rect[obj_num]);
		else if(vector_rect[obj_num].area() > 1500)
			distance_vector[2].push_back(vector_rect[obj_num]);
		else if(vector_rect[obj_num].area() > 900)
			distance_vector[3].push_back(vector_rect[obj_num]);
		else if(vector_rect[obj_num].area() > 600)
			distance_vector[4].push_back(vector_rect[obj_num]);
		else if(vector_rect[obj_num].area() > 400)
			distance_vector[5].push_back(vector_rect[obj_num]);
	}
	for(int obj_num = 0; obj_num < distance_vector.size(); obj_num ++)
	{
		if(distance_vector[obj_num].size() != 0)
		{
			cout << "select obj_num : "<< obj_num << endl;
			return distance_vector[obj_num];
		    //break;
		}
	}
	vector<Rect> tmp_none_rect;
	return tmp_none_rect;
}*/

Point armour::select_the_rect(Mat &src_image, vector<Rect> &vector_rect) {
    double min_distance;
    static Rect prev_rect;
    //cout << "debug_size:" << vector_rect.size() << endl;
    static Point prev_point;
    register double max_area = 0;
    register int tmp_index = 0;
    register Point tmp_point = ERROR_POINT;
    vector_rect = get_the_distance_vector(vector_rect);
    vector<Rect> proper_rect;
    register double height_width_proportion;
    if (vector_rect.size() == 0) return tmp_point;
    //vector_rect = sort_area(vector_rect);
    max_area = vector_rect[0].area();
    for (int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {
        height_width_proportion = (double) vector_rect[obj_num].height / vector_rect[obj_num].width;
        if (height_width_proportion > 0.41 && height_width_proportion < 0.47) {
            proper_rect.push_back(vector_rect[obj_num]);
        }
    }
    /*if(proper_rect.size() != 0)
    {
        for(int obj_num = 0;obj_num < proper_rect.size();obj_num ++)
        {
            //cout << "blueblue" <<endl;
            if(max_area > proper_rect[obj_num].area())
            {
                max_area = proper_rect[obj_num].area();
                tmp_point = Point(proper_rect[obj_num].x + proper_rect[obj_num].width/2.0,proper_rect[obj_num].y + proper_rect[obj_num].height/2.0);
                tmp_index = obj_num;
            }
        }
        prev_point = tmp_point;
        first_select_flags = true;
        rectangle(src_image,proper_rect[tmp_index],Scalar(255,0,255),2,8);
        return tmp_point;
    }*/
    if (!first_select_flags) {
        for (int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {
            //cout << "balabala" << endl;
            if (vector_rect[obj_num].area() < max_area) {
                tmp_point = Point(vector_rect[obj_num].x + vector_rect[obj_num].width / 2.0,
                                  vector_rect[obj_num].y + vector_rect[obj_num].height / 2.0);
                max_area = vector_rect[obj_num].area();
                tmp_index = obj_num;
            }
        }
        first_select_flags = true;
        prev_point = tmp_point;
        prev_rect = vector_rect[tmp_index];
        rectangle(src_image, vector_rect[tmp_index], Scalar(0, 0, 255), 2, 8);
        distance_single = CalculateZ(0, vector_rect[tmp_index].height, 60, focus_global);
        return tmp_point;
    }
    vector<double> distance;
    for (register int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {
        distance.push_back(sqrt(pow(320 - vector_rect[obj_num].x, 2) + pow(240 - vector_rect[obj_num].y, 2) +
                                0.3 * pow(vector_rect[obj_num].x - prev_point.x, 2) +
                                0.3 * pow(vector_rect[obj_num].y - prev_point.y, 2) +
                                0.2 * pow(prev_rect.width - vector_rect[obj_num].width, 2) +
                                0.2 * pow(prev_rect.height - vector_rect[obj_num].height, 2)));
    }
    if (distance.size() < 5) {
        //vector<Rect> sort_rect = bubble_sort(distance,vector_rect);
        register double min_area = vector_rect[0].area();
        //register double min_distance = distance[0];
        register int min_index = 0;

        for (register int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {

            if (min_area > vector_rect[obj_num].area()) {
                min_area = vector_rect[obj_num].area();
                min_index = obj_num;
            }
            /*if(min_distance > distance[obj_num])
            {
                min_distance = distance[obj_num];
                min_index = obj_num;
            }*/
        }
        min_distance = sqrt(pow(prev_point.x - vector_rect[min_index].x - vector_rect[min_index].width / 2.0, 2) +
                            pow(prev_point.y - vector_rect[min_index].y - vector_rect[min_index].height / 2.0, 2));
        rectangle(src_image, vector_rect[min_index], Scalar(0, 255, 0), 3.86);
        circle(src_image, prev_point, 2, Scalar(0, 255, 255), 3.6);
        circle(src_image, Point(vector_rect[min_index].x + vector_rect[min_index].width / 2.0,
                                vector_rect[min_index].y + vector_rect[min_index].height / 2.0), 2, Scalar(0, 255, 0),
               2, 8);
        cout << "distance : "
             << sqrt(pow(prev_point.x - vector_rect[min_index].x - vector_rect[min_index].width / 2.0, 2) +
                     pow(prev_point.y - vector_rect[min_index].y - vector_rect[min_index].height / 2.0, 2)) << endl;
        //display("debug" , src_image);
        //waitKey();
        /*	if(min_distance > 150)
            {
                distance_times ++;
                if(distance_times> 20)
                {
                    distance_times = 0;
                    prev_point = Point(vector_rect[min_index].x +vector_rect[min_index].width / 2.0,vector_rect[min_index].y + vector_rect[min_index].height/2.0);
                    prev_rect = vector_rect[min_index];
                }
                distance_single = CalculateZ(0,prev_rect.height,60,focus_global);
                return prev_point;
            }*/
        distance_times = 0;
        //	cout << "debug" << endl;
        //double min_distance =  sqrt(pow(prev_point.x - vector_rect[min_index].x - vector_rect[min_index].width / 2.0,2) + pow(prev_point.y - vector_rect[min_index].y - vector_rect[min_index].height/2.0,2));
        prev_point = Point(vector_rect[min_index].x + vector_rect[min_index].width /
                                                      2.0 /*+ (vector_rect[min_index].x + vector_rect[min_index].width / 2.0 - prev_point.x)*/,
                           vector_rect[min_index].y + vector_rect[min_index].height /
                                                      2.0 /*+ (vector_rect[min_index].y + vector_rect[min_index].height / 2.0 - prev_point.y)*/);
        //prev_point = Point(sort_rect[0].x + sort_rect[0].width / 2.0,sort_rect[0].y + sort_rect[0].height / 2.0);
        distance_single = CalculateZ(0, vector_rect[min_index].height, 60, focus_global);
        return prev_point;
    }
    register int min_index;
    register double min_area;
    vector<Rect> sort_rect = bubble_sort(distance, vector_rect);
    min_area = sort_rect[0].area();
    double max_area_tmp = get_max_area(sort_rect);
    //cout << "max_area:" << max_area_tmp << endl;
    for (register int obj_num = 0; obj_num < 2; obj_num++) {
        if (min_area > sort_rect[obj_num].area()) {

            /*if(sort_rect[obj_num].area() < max_area_tmp/1.5)
                continue;*/
            min_area = sort_rect[obj_num].area();
            //cout << "min_area : "<< min_area << " obj_area:" << sort_rect[obj_num].area() << endl;
            min_index = obj_num;
        }
    }
    min_distance = sqrt(pow(prev_point.x - sort_rect[min_index].x - sort_rect[min_index].width / 2.0, 2) +
                        pow(prev_point.y - sort_rect[min_index].y - sort_rect[min_index].height / 2.0, 2));
    rectangle(src_image, sort_rect[min_index], Scalar(255, 255, 0), 2, 8);
    circle(src_image, prev_point, 2, Scalar(0, 255, 255), 2, 8);
    circle(src_image, Point(sort_rect[min_index].x + sort_rect[min_index].width / 2.0,
                            sort_rect[min_index].y + sort_rect[min_index].height / 2.0), 2, Scalar(0, 255, 0), 2, 8);
    //cout << "distance : "<< sqrt(pow(prev_point.x - sort_rect[min_index].x - sort_rect[min_index].width / 2.0,2) + pow(prev_point.y - sort_rect[min_index].y - sort_rect[min_index].height/2.0,2)) <<endl;
    //display("debug" , src_image);
    //waitKey();
    if (min_distance > 150) {
        distance_times++;
        if (distance_times > 20) {
            distance_times = 0;
            prev_rect = sort_rect[min_index];
            prev_point = Point(sort_rect[min_index].x + sort_rect[min_index].width / 2.0,
                               sort_rect[min_index].y + sort_rect[min_index].height / 2.0);
        }
        distance_single = CalculateZ(0, prev_rect.height, 60, focus_global);
        return prev_point;
    }
    distance_times = 0;
    prev_point = Point(sort_rect[min_index].x + sort_rect[min_index].width /
                                                2.0  /*+ (sort_rect[min_index].x + sort_rect[min_index].width / 2.0 - prev_point.x)*/,
                       sort_rect[min_index].y + sort_rect[min_index].height /
                                                2.0 /*+ (sort_rect[min_index].y + sort_rect[min_index].height / 2.0 - prev_point.y)*/);
    prev_rect = sort_rect[min_index];
    distance_single = CalculateZ(0, sort_rect[min_index].height, 60, focus_global);
    //cout << "area: " << min_area << endl;

    //return prev_point
    //waitKey();


    return prev_point;
    //return prev_point;
}

Point armour::select_the_rect_rotate(Mat &src_image, vector<Rect> &vector_rect, int &armour_area, Rect &select_rect) {
    static int flags_count = 0;
    int length_vector = vector_rect.size();
    if (length_vector == 0) return ERROR_POINT;
    for (int obj_num = 0; obj_num < length_vector; obj_num++) {
        //for(int obj_num_index = 10; obj_num_index >= 0; obj_num_index--)
        //{
        //if(vector_rect[obj_num].contains(Point(320 - obj_num_index,0)) || vector_rect[obj_num].contains(Point(320 + obj_num_index,0)))
        if (vector_rect[obj_num].contains(Point(320, 240))) {
            flags_count = 0;
            return ERROR_POINT;
        }

    }
    if (flags_count <= 2) return Point(320, 240);
    flags_count++;
    double min_distance = DBL_MAX;
    Point return_point;
    for (int obj_num = 0; obj_num < length_vector; obj_num++) {
        Point center_of_rect = Point(vector_rect[obj_num].x + vector_rect[obj_num].width / 2.0,
                                     vector_rect[obj_num].y + vector_rect[obj_num].height / 2.0);
        double distance_tmp = sqrt(pow(center_of_rect.x - 320, 2) + pow(center_of_rect.y - 240, 2));
        if (min_distance > distance_tmp) {
            min_distance = distance_tmp;
            select_rect = vector_rect[obj_num];
            distance_single = CalculateZ(0, vector_rect[obj_num].height, 60, focus_global);
            return_point = center_of_rect;
        }
    }
    return return_point;
}

Point armour::select_the_rect(Mat &src_image, vector<Rect> &vector_rect, int &return_area, Rect &return_rect) {
    double min_distance;
    static Rect prev_rect;
    //cout << "debug_size:" << vector_rect.size() << endl;
    static Point prev_point = ERROR_POINT;
    register double max_area = 0;
    register int tmp_index = 0;
    register Point tmp_point = ERROR_POINT;
    //vector_rect = get_the_distance_vector(vector_rect);
    vector<Rect> proper_rect;
    register double height_width_proportion;
    //cout << "prev_rect  -> width :" << prev_rect.width << "height : "<< prev_rect.height << endl;
    static int no_target = 0;
    if (vector_rect.size() == 0 && first_select_flags != false) //not the first and never find the rect
    {
        target_flags = false;
        no_target++;
        if (no_target <= 1) {
            return_area = prev_rect.area();
            distance_single = CalculateZ(0, prev_rect.height, 60, focus_global);
            return_rect = prev_rect;
            return prev_point;
        }

        return tmp_point;
    } else if (vector_rect.size() == 0 && first_select_flags == false)//first run this program and never find the target
    {
        target_flags = false;
        return_area = 0;
        return ERROR_POINT;
    }//vector_rect = sort_area(vector_rect);
    no_target = 0;
    max_area = vector_rect[0].area();
    for (int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {
        height_width_proportion = (double) vector_rect[obj_num].height / vector_rect[obj_num].width;
        if (height_width_proportion > 0.41 && height_width_proportion < 0.47) {
            proper_rect.push_back(vector_rect[obj_num]);
        }
    }
    /*if(proper_rect.size() != 0)
    {
        for(int obj_num = 0;obj_num < proper_rect.size();obj_num ++)
        {
            //cout << "blueblue" <<endl;
            if(max_area > proper_rect[obj_num].area())
            {
                max_area = proper_rect[obj_num].area();
                tmp_point = Point(proper_rect[obj_num].x + proper_rect[obj_num].width/2.0,proper_rect[obj_num].y + proper_rect[obj_num].height/2.0);
                tmp_index = obj_num;
            }
        }
        prev_point = tmp_point;
        first_select_flags = true;
        rectangle(src_image,proper_rect[tmp_index],Scalar(255,0,255),2,8);
        return tmp_point;
    }*/
    if (!first_select_flags) {
        for (int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {
            //cout << "balabala" << endl;
            if (vector_rect[obj_num].area() < max_area) {
                tmp_point = Point(vector_rect[obj_num].x + vector_rect[obj_num].width / 2.0,
                                  vector_rect[obj_num].y + vector_rect[obj_num].height / 2.0);
                max_area = vector_rect[obj_num].area();
                tmp_index = obj_num;
            }
        }
        first_select_flags = true;
        prev_point = tmp_point;
        prev_rect = vector_rect[tmp_index];
        rectangle(src_image, vector_rect[tmp_index], Scalar(0, 0, 255), 2, 8);
        distance_single = CalculateZ(0, vector_rect[tmp_index].height, 60, focus_global);
        return_area = prev_rect.area();
        return_rect = prev_rect;
        //distance_single = CalculateZ(vector_rect[tmp_index]);
        return tmp_point;
    }
    vector<double> distance;
    for (register int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {
        double Roi_width_bytes_height = (double) vector_rect[obj_num].width / vector_rect[obj_num].height - 0.42;
        distance.push_back(sqrt(1 * pow(Roi_width_bytes_height, 2) + 0.3 * pow(320 - vector_rect[obj_num].x, 2) +
                                0.3 * pow(240 - vector_rect[obj_num].y, 2) +
                                0.3 * pow(vector_rect[obj_num].x - prev_point.x, 2) +
                                0.3 * pow(vector_rect[obj_num].y - prev_point.y, 2) +
                                0.8 * pow(prev_rect.width - vector_rect[obj_num].width, 2) +
                                0.8 * pow(prev_rect.height - vector_rect[obj_num].height, 2)));
    }
    if (distance.size() < 2) {
        //vector<Rect> sort_rect = bubble_sort(distance,vector_rect);
        register double min_area = vector_rect[0].area();
        //register double min_distance = distance[0];
        register int min_index = 0;

        for (register int obj_num = 0; obj_num < vector_rect.size(); obj_num++) {

            if (min_area > vector_rect[obj_num].area()) {
                min_area = vector_rect[obj_num].area();
                min_index = obj_num;
            }
            /*if(min_distance > distance[obj_num])
            {
                min_distance = distance[obj_num];
                min_index = obj_num;
            }*/
        }
        min_distance = sqrt(pow(prev_point.x - vector_rect[min_index].x - vector_rect[min_index].width / 2.0, 2) +
                            pow(prev_point.y - vector_rect[min_index].y - vector_rect[min_index].height / 2.0, 2));

        rectangle(src_image, vector_rect[min_index], Scalar(0, 255, 0), 3.86);
        prev_rect = vector_rect[min_index];
        circle(src_image, prev_point, 2, Scalar(0, 255, 255), 3.6);
        circle(src_image, Point(vector_rect[min_index].x + vector_rect[min_index].width / 2.0,
                                vector_rect[min_index].y + vector_rect[min_index].height / 2.0), 2, Scalar(0, 255, 0),
               2, 8);
        cout << "distance : "
             << sqrt(pow(prev_point.x - vector_rect[min_index].x - vector_rect[min_index].width / 2.0, 2) +
                     pow(prev_point.y - vector_rect[min_index].y - vector_rect[min_index].height / 2.0, 2)) << endl;
        //display("debug" , src_image);
        //waitKey();
/*        if(min_distance > 300)
		{
			distance_times ++;
            if(distance_times> 2 )
			{
				distance_times = 0;
				prev_point = Point(vector_rect[min_index].x +vector_rect[min_index].width / 2.0,vector_rect[min_index].y + vector_rect[min_index].height/2.0);
				prev_rect = vector_rect[min_index];
			}
			distance_single = CalculateZ(0,prev_rect.height,60,focus_global);
			//distance_single = CalculateZ(prev_rect);
			return_area = prev_rect.area();
			return_rect = prev_rect;
			return prev_point;
        }*/
        distance_times = 0;
        //	cout << "debug" << endl;
        //double min_distance =  sqrt(pow(prev_point.x - vector_rect[min_index].x - vector_rect[min_index].width / 2.0,2) + pow(prev_point.y - vector_rect[min_index].y - vector_rect[min_index].height/2.0,2));
        prev_point = Point(vector_rect[min_index].x + vector_rect[min_index].width /
                                                      2.0 /*+ (vector_rect[min_index].x + vector_rect[min_index].width / 2.0 - prev_point.x)*/,
                           vector_rect[min_index].y + vector_rect[min_index].height /
                                                      2.0 /*+ (vector_rect[min_index].y + vector_rect[min_index].height / 2.0 - prev_point.y)*/);
        //prev_point = Point(sort_rect[0].x + sort_rect[0].width / 2.0,sort_rect[0].y + sort_rect[0].height / 2.0);
        distance_single = CalculateZ(0, vector_rect[min_index].height, 60, focus_global);
        //	distance_single = CalculateZ(vector_rect[min_index]);
        return_area = prev_rect.area();
        return_rect = prev_rect;
        return prev_point;
    }
    register int min_index;
    register double min_area;
    vector<Rect> sort_rect = bubble_sort(distance, vector_rect);
    min_area = sort_rect[0].area();
    double max_area_tmp = get_max_area(sort_rect);
    //cout << "max_area:" << max_area_tmp << endl;
    for (register int obj_num = 0; obj_num < 2; obj_num++) {
        if (min_area > sort_rect[obj_num].area()) {

            /*if(sort_rect[obj_num].area() < max_area_tmp/1.5)
                continue;*/
            min_area = sort_rect[obj_num].area();
            //cout << "min_area : "<< min_area << " obj_area:" << sort_rect[obj_num].area() << endl;
            min_index = obj_num;
        }
    }
    min_distance = sqrt(pow(prev_point.x - sort_rect[min_index].x - sort_rect[min_index].width / 2.0, 2) +
                        pow(prev_point.y - sort_rect[min_index].y - sort_rect[min_index].height / 2.0, 2));
    rectangle(src_image, sort_rect[min_index], Scalar(255, 255, 0), 2, 8);
    circle(src_image, prev_point, 2, Scalar(0, 255, 255), 2, 8);
    circle(src_image, Point(sort_rect[min_index].x + sort_rect[min_index].width / 2.0,
                            sort_rect[min_index].y + sort_rect[min_index].height / 2.0), 2, Scalar(0, 255, 0), 2, 8);

//	cout << "distance : "<< sqrt(pow(prev_point.x - sort_rect[min_index].x - sort_rect[min_index].width / 2.0,2) + pow(prev_point.y - sort_rect[min_index].y - sort_rect[min_index].height/2.0,2)) <<endl; 
//	display("debug" , src_image);
//	waitKey();

    if (min_distance > 300) {
        distance_times++;
        if (distance_times > 5) {
            distance_times = 0;
            prev_rect = sort_rect[min_index];
            prev_point = Point(sort_rect[min_index].x + sort_rect[min_index].width / 2.0,
                               sort_rect[min_index].y + sort_rect[min_index].height / 2.0);
            prev_rect = sort_rect[min_index];
        }
        distance_single = CalculateZ(0, prev_rect.height, 60, focus_global);
        //	distance_single = CalculateZ(prev_rect);
        return_area = prev_rect.area();
        return_rect = prev_rect;
        return prev_point;
    }
    distance_times = 0;
    prev_point = Point(sort_rect[min_index].x + sort_rect[min_index].width /
                                                2.0  /*+ (sort_rect[min_index].x + sort_rect[min_index].width / 2.0 - prev_point.x)*/,
                       sort_rect[min_index].y + sort_rect[min_index].height /
                                                2.0 /*+ (sort_rect[min_index].y + sort_rect[min_index].height / 2.0 - prev_point.y)*/);
    prev_rect = sort_rect[min_index];
    distance_single = CalculateZ(0, sort_rect[min_index].height, 60, focus_global);
//	distance_single = CalculateZ(sort_rect[min_index]);
    //cout << "area: " << min_area << endl;

    //return prev_point
    //waitKey();

    return_area = prev_rect.area();
    return_rect = prev_rect;
    return prev_point;
    //return prev_point;
}

//function for write pic
string armour::change_int_into_path_jpg(string folder, int num) {
    string tmp_path;
    //cout << "enter get_num_name func" << endl;
    stringstream str_stream;
    string num_str;
    str_stream << num;
    str_stream >> num_str;
    /*if (num / 10.0 < 1)
        num_str = "000000" + num_str;
    else if (num / 100.0 < 1)
        num_str = "00000" + num_str;
    else if (num / 1000.0 < 1)
        num_str = "0000" + num_str;
    else if (num / 10000.0 < 1*/
    tmp_path = folder + num_str + ".avi";
//cout << "quit" << endl;
    return tmp_path;
}

int armour::return_the_exsit_num(string folder) {
    vector<String> file_names;
    glob(folder, file_names);
    return file_names.size();
}

static void *get_fram_thread_1(void *arg) {
    Mat *src_image = (Mat *) arg;
    VideoCapture cap_thread(0);
    if (cap_thread.isOpened() == false) {
        cout << "can't get the fram thread" << endl;
        exit(-1);
    }
    vector<Mat> pic_array_thread;
    int thread_flags = 0;
    while (1) {
        if (true == process_finish && thread_flags != 0) {
            /*if(NULL == pic_array_thread.back().data)
            {
                 cap >> *src_image;
                 process_finish = false;
            }*/
            pic_array_thread.back().copyTo(*src_image);
            pic_array_thread.clear();
            process_finish = false;
        }
        if (true == thread_lauch_first) {
            Mat fram;
            cap_thread >> fram;
            fram.copyTo(*src_image);
            thread_lauch_first = false;
            continue;
        }
        Mat fram_temp;
        cap_thread >> fram_temp;
        if (fram_temp.data == NULL) {
            cout << "fram can't access" << endl;
            exit(-1);
        }
        pic_array_thread.push_back(fram_temp);
        thread_flags++;
    }
}

static void *get_fram_thread(void *arg) {
    Mat *src_image = (Mat *) arg;
    vector<Mat> pic_array_thread;

    int serial_camera = get_usb_camara_serial(BIG_OR_SMALL);
    V4L2DeviceParameters param("/dev/video0", V4L2_PIX_FMT_MJPEG, IMAGE_WIDTH, IMAGE_HEIGHT, FPS);
    if (serial_camera != 0) {
        if (serial_camera == 1) {
            param.m_devName = "/dev/video1";
        }
    } else {
//        cap_thread = set_v4l("/dev/video0");
    }

    V4l2Capture *cap_thread = V4l2Capture::create(param);

    int thread_flags = 0;
    while (1) {
        if (true == process_finish && thread_flags != 0) {
            /*if(NULL == pic_array_thread.back().data)
            {
                 cap >> *src_image;
                 process_finish = false;
            }*/
            pic_array_thread.back().copyTo(*src_image);
            pic_array_thread.clear();
            process_finish = false;
        }
        if (true == thread_lauch_first) {
            Mat fram;
            timeval timeout{};
            bool isReadable = cap_thread->isReadable(&timeout);
            if (!isReadable) {
                cout << "Is not Readable" << endl;
            }
            isReadable = cap_thread->read(fram);

            fram.copyTo(*src_image);
            thread_lauch_first = false;
            process_finish = false;
            sem_post(&finish_sem);
            continue;
        }
        Mat fram_temp;
        cap_thread->read(fram_temp);
        // VideoPlayer(fram);
        if (fram_temp.data == NULL) {
            cout << "fram can't access" << endl;
            exit(-1);
        }
        pic_array_thread.push_back(fram_temp);
        thread_flags++;
    }
    cap_thread->stop();
    delete cap_thread;
}

int main() {
    //init the thread
    pthread_t Read_Uart;
    pthread_t Send_Uart;
    pthread_t led_thread;
    pthread_t get_fram;
    int ret;

    //init the sem
    sem_init(&finish_sem, 0, 0);//init the signal
    sem_init(&first_sem, 0, 1);

    //usleep(1000*10000);
    //sem_init(&sem_main,0,0);//	busy
    //sem_init(&sem_vedio,0,1);

    //! 重构
    if (serialInit() == false){
        cout << "serial init fail" << endl;
//        return 0;
    }
    ret = pthread_create(&Read_Uart, NULL, serialRead, NULL);
    if (ret != 0) {
        cout << "read_fail" << endl;
    }
    ret = pthread_create(&Send_Uart, NULL, serialSend, NULL);
    ret = pthread_create(&led_thread,NULL, led_on_thread, NULL);
    if (ret != 0) {
        std::cout << "led_fail" << std::endl;
    }
    //妙算只能跑这个分辨率
//    Mat org_image(Size(1440, 1080), CV_8UC3); //原始分辨率
    Mat src_image(Size(640, 480), CV_8UC3); //运算分辨率


#ifdef thread_lauch
    ret = pthread_create(&get_fram,NULL,get_fram_thread,(void *)&src_image);
    if(ret != 0)
    {
        cout << "get_fram thread can't lauch" << endl;
        exit(-1);
    }
#else
    //视频信号输出
    VideoCapture cap;//  int USB_Camara_serial = get_usb_camara_serial(BIG_OR_SMALL);
    cap.open(1);
    //cap.set(CV_CAP_PROP_AUTO_EXPOSURE,0.25);


    //cvGetCaptureProperty(&cap,CV_CAP_PROP_EXPOSURE);
    //cout << "exp:" << cap.set(CV_CAP_PROP_EXPOSURE,1) << endl;
    /*V4L2Capture * cap_thread;
    int serial_camera = get_usb_camara_serial(BIG_OR_SMALL);
    if(serial_camera == 0)
        cap_thread  =  set_v4l("/dev/video0");
    else if(serial_camera == 1)
        cap_thread = set_v4l("/dev/video1");
    else
        cap_thread  =  set_v4l("/dev/video0"); */
#endif
    armour auto_beat(0);
    //buff_wu
    //Ligth buf_w("buf");
//	Detect_obj buf;
    //endif
    bool first_flags = false;
    //int val_min = 0;
    //int val_max = 0;
    //int h_min,h_max,s_min,s_max,v_min,v_max;
    //namedWindow("image show", 1);
    //createTrackbar("h_min", "image show", &val_min, 255, mytrackBarCallback, &h_min);
    //createTrackbar("h_max", "image show", &val_max, 255, mytrackBarCallback, &h_max);
    //createTrackbar("s_min", "image show", &val_min, 255, mytrackBarCallback, &s_min);
    //createTrackbar("s_max", "image show", &val_max, 255, mytrackBarCallback, &s_max);
    //createTrackbar("v_min", "image show", &val_min, 255, mytrackBarCallback, &v_min);
    //createTrackbar("v_max", "image show", &val_max, 255, mytrackBarCallback, &v_max);

    /// 无尽循环开始
    for (;;) {

        //double cost_time = get_sys_time();
/// 没有定义
#ifdef thread_lauch

        process_finish = true;
        if(first_flags == false)
        {
            first_flags = true;
            sem_wait(&finish_sem);
        } /*while(thread_lauch_first)
        {
            process_finish = false;
        }*/
        while(process_finish);
#else

        //src_image = VideoCapture_v4l(cap_thread);
        //double cost_time = get_sys_time();
        auto_beat.cost_time = get_sys_time();
//        cap >> org_image;
        //cut image
//        Rect rect_org(400, 300, 640, 480);
//        src_image = org_image(rect_org);
//        resize(org_image, src_image, Size(640, 480));
        cap >> src_image;

        //Mat HSV_to_gray(const Mat & src_image,int mode,int &h_min,int &h_max ,int &s_min,int &s_max ,int &v_min ,int &v_max )
        //Mat dst_image = HSV_to_gray(src_image,h_min,h_max,s_min,s_max,v_min,v_max);
        //cout << "h_max:" << h_max << endl;
        //cost_time = (get_sys_time() - cost_time) / getTickFrequency();
        //cout << "fps:" << 1./cost_time << endl;
        //continue;
        //imshow("image",src_image);
        //waitKey();

        //imshow("image show",dst_image);
        //waitKey(5);
        //continue;
#endif
        // display("debug",src_image);
        // MOD_B_R = 1;
        //cout << "src read" << endl;
        //display("src",src_image);
        if (src_image.data == NULL) {
            cout << "wrong read_data" << endl;
        }

        //装甲板
        if (MOD_B_R != 3) {
            cout << "auto beat" << endl;
            //串口数据在fire中发送
            auto_beat.fire(src_image);
        }

        //能量机关？
        if (MOD_B_R == 3) {

            Rect buf_out_rect;
            double never_used_weights = 0;
            //cap >> src_image;
            //if(src_image.data == NULL) cout << "wrong" << endl;
            //imshow("debug",src_image);
            //cap.release();
            Point2f buf_out_point;
            //buf_w.Dafu_Out(src_image,false,buf_out_rect,never_used_weights,buf_out_point);
            //	display("bufWu_Pic",src_image);
            //	cout << "debug .. " << endl;
            cout << "buf_point (" << buf_out_point.x << "," << buf_out_point.y << ")" << endl;
            vector<double> angle_vector = Calculate_angle_buf(Point(0, 0),
                                                              Point(buf_out_point.x - 320, buf_out_point.y - 240),
                                                              800.0);
            if (ERROR_POINT_BUF == buf_out_point) {
                //serialPosData(angle_vector[0] * 1000, angle_vector[1] * 1000, 1, 0, 0);
                serialPosData(angle_vector[0] * 1000, angle_vector[1] * 1000, 1, 0);
                setDataCmd(0);
                //	cout << "debug ... " << endl;
            }
            cout << "yaw :" << angle_vector[0] << " , ph:" << angle_vector[1] << endl;
            //	std::cout << "enter big buf mode" << std::endl;
            //serialPosData(angle_vector[0] * 1000, angle_vector[1] * 1000, 1, 0, 1);
            serialPosData(angle_vector[0] * 1000, angle_vector[1] * 1000, 1, 0);
            setDataCmd(1);
            cout << "debug ..." << endl;
        }

    }
    //无尽循环结束
    //cap_thread->stopCapture();
    //cout <<cap_thread->freeBuffers();
    //cout <<cap_thread->closeDevice();
    pthread_join(get_fram, NULL);
    pthread_join(Read_Uart, NULL);
    printf("main 2\r\n");
    pthread_join(Send_Uart, NULL);
    //pthread_join(led_thread,NULL);
    printf("main 3\r\n");
    return 0;
}

vector<Rect> armour::judge_contour(Mat src_image, const vector<Rect> &rect_vector) {
    int circle_num = 0;
    vector<int> contour_num_vector;
    int count_contour = 0;
    cout << "size : " << rect_vector.size() << endl;
    //cvtColor(src_image,src_image,CV_BGR2GRAY);
    for (int obj_num = 0; obj_num < rect_vector.size(); obj_num++) {
        //cout << "src_size: " << src_image.rows << "src_height : "<< src_image.cols << endl;
        //cout << "rect_vector.width" << rect_vector[obj_num].x  << "," << rect_vector[obj_num].width+rect_vector[obj_num].x << endl;
        //cout << "rect_vector.height:" << rect_vector[obj_num].y << "," << rect_vector[obj_num].height + rect_vector[obj_num].y << endl;
        if (rect_vector[obj_num].height == 0) {
            //cout << "wrong" << endl;
            //exit( -1 );
            circle_num++;
            continue;
        }
        if (rect_vector[obj_num].width == 0) {
            circle_num++;
            continue;
        }
        Mat ROI_pic = src_image(rect_vector[obj_num]);
        vector<vector<Point>> contour_vector;
        vector<Vec4i> contour_hierachy;
        cout << "hello" << endl;
        findContours(ROI_pic, contour_vector, contour_hierachy, RETR_EXTERNAL, CHAIN_APPROX_TC89_L1);
        //CV_LINK_RUNS may can be used in cv4
        //imshow("roi_pic",ROI_pic);

        cout << "contour size:" << contour_vector.size() << endl;
        //waitKey();
        count_contour += contour_vector.size();
        contour_num_vector.push_back(contour_vector.size());
        //cout << "contour_size:" << contour_vector.size() << endl;
    }

    double ave = (double) count_contour / (rect_vector.size() - circle_num);
    cout << "ave : " << ave << endl;
    cout << "size : " << rect_vector.size() << endl;
    /*double tmp_stddev = 0;
    for(int obj_num = 0;obj_num < contour_num_vector.size();obj_num++)
    {
        tmp_stddev += pow(contour_num_vector[obj_num] - ave,2);
    }
    double stddev = sqrt(tmp_stddev/(contour_num_vector.size()));
    double normal = stddev + ave;
    //if(contour_vector.size() > 4) return false;*/
    vector<Rect> dst_vector_rect;
    for (int obj_num = 0; obj_num < contour_num_vector.size(); obj_num++) {
        if (contour_num_vector[obj_num] <= 1.0 * ave) {
            cout << "obj_num : " << obj_num << "rect_vector.size()" << rect_vector.size() << endl;
            dst_vector_rect.push_back(rect_vector[obj_num]);
        }
        //cout << "debug" << endl;
    }
//	cout << "debug" << endl;
    //cout << "ave:" << ave << " stddev: "<< stddev << " normal :" << normal << endl;
    return dst_vector_rect;
    //return true;
}

void *armour::collect_pic_thread(void *arg) {
    cout << "meximexi" << endl;
    armour *_this = (armour *) arg;
    //Mat src_image
    VideoWriter out;
    int exist_num = _this->return_the_exsit_num(video_path);
    string video_full_path = _this->change_int_into_path_jpg(video_path, exist_num);
    out.open(video_full_path, CAP_OPENCV_MJPEG, 40, Size(640, 480), true);
    while (true) {
        /*if(-> collect_image.data != NULL)
        {
            cout  << "debug" << endl;
            out << _this -> collect_image;
        }*/
        if (_this->collect_flags == true) {
            cout << "debug" << endl;
            out << _this->collect_image;
        }

    }
}


Point select_kalman_or_real(Point target_kalman_point, Point real_predict) {
    cout << "target_sub:" << fabs(target_kalman_point.x - real_predict.x) << endl;
    if (fabs(target_kalman_point.x - real_predict.x) > 0.0 && fabs(target_kalman_point.x - real_predict.x) < 20.0) {
        cout << "kalman " << endl;
        return target_kalman_point;
    }
    cout << "src" << endl;
    return target_kalman_point;
}


Rect armour::get_new_roi_rect(bool flags) {
    //flags = false;
    Rect roi_rect;
    int Roi_width;
    if (distance_single < 0.5)
        Roi_width = 280;
    else
        Roi_width = Polyfit_Get(distance_single) + 20;
    int Roi_height = 150;
    flags = false;
    cout << "roi_width : " << Roi_width << endl;
    switch (flags) {
        case true:
            roi_rect.x = 320 - Roi_width / 2.0;
            roi_rect.y = 240 - Roi_height / 2.0;
            roi_rect.width = Roi_width;
            roi_rect.height = Roi_height;
            break;
        case false:
            roi_rect.x = 0;
            roi_rect.y = 0;
            roi_rect.width = 640;
            roi_rect.height = 480;
            break;
    }
    return roi_rect;
}

void armour::resize_Point(Rect normal_rect, vector<Rect> &point_rect) {
    for (int obj_num = 0; obj_num < point_rect.size(); obj_num++) {
        point_rect[obj_num] = Rect(point_rect[obj_num].x + normal_rect.x, point_rect[obj_num].y + normal_rect.y,
                                   point_rect[obj_num].width, point_rect[obj_num].height);
    }
}

void create_test_vector(vector<Point> &test) {
    for (int obj_num = 0; obj_num < 3; obj_num++) {
        test.push_back(Point(320, 240));
    }
}


void armour::select_rect_by_angle(const Rect &select_rect, vector<double> &angle_vector, double distance) {
    double y_angle = angle_vector[1];
    if (fabs(angle_vector[0]) < 1.0);
    else if (angle_vector[0] > 1.0)
        angle_vector = Calculate_angle(Point(0, 0), Point(select_rect.x - 320, select_rect.y - 240), distance);
    else if (angle_vector[0] < -1.0)
        angle_vector = Calculate_angle(Point(0, 0), Point(select_rect.x + select_rect.width - 320,
                                                          select_rect.y + select_rect.height - 240), distance);
    angle_vector[1] = y_angle;
}


void armour::fire(Mat &src_image) {
    pthread_t collect_thread;
    //int notarget_lable=0;
    static Point prev_point_fire = Point(0, 0);
    this->collect_flags = false;
    ofstream file_stream;
    //AntiKalman::KalmanFilter kf(0,0);
    //double no_target_label=0;
    double prev_distance = 0;
    AntiKalman::KalmanFilter kf(320, 240);
    //pthread_create(&collect_thread,NULL,collect_pic_thread,(void *)this);
    //waitKey(100);
    /*int src_exist_num = 0;
    int dst_exist_num = 0;
    int gray_exist_num = 0;*/
    //write_pic:
    //int src_exist_num = return_the_exsit_num(video_path);
    /*int dst_exist_num = return_the_exsit_num(dst_data_dir);
    int gray_exist_num = return_the_exsit_num(gray_data_dir);*/
    //string video_full_path = change_int_into_path_jpg(video_path,src_exist_num);
    //VideoWriter writer;
    //writer.open(video_full_path,CV_FOURCC('M','J','P','G'),40,Size(640,480),true);
    //file_stream.open("./record.txt",ios::out);
    vector<double> angle_vector;
    vector<Point> test;
    Rect Roi_rect;
    //while (MOD_B_R != 3)
    {
//      mode = MODE_BLUE;

        //target_flags = false;?
        Roi_rect = get_new_roi_rect(target_flags);

        src_image = src_image(Roi_rect);

        if (!src_image.data) {
            cout << "read data failed .. " << endl;
            exit(-1);
        }
        Mat gray_image = HSV_to_gray(src_image, BEAT_COLOR);
        //gray_image = erode_pic(gray_image, 1);
        gray_image = dilate_pic(gray_image, 12);
        //display("gray", gray_image);

        vector<vector<Point>> vector_Point;
        get_rect_pic_contour(gray_image, get_point_contours(gray_image), vector_Point);
        //vector_Point = judge_the_min_distance(gray_image,vector_Point);
        //cout << "vector_point size:" << vector_Point.size() << endl;
        vector<double> angle_vector;

        angle_vector = get_rotate_angle(src_image, vector_Point);
        vector<Scalar> place_info_vector;
        place_info_vector = judge_the_theata(cal_distance_theata(angle_vector), 3);


        vector<Rect> Rect_vector;
        set_the_rect(gray_image, place_info_vector, vector_Point, Rect_vector, src_image.rows, src_image.cols);
        int armour_area;
        Mat result_pic = rect_the_pic(gray_image, src_image, Rect_vector);
        resize_Point(Roi_rect, Rect_vector);
        Rect select_rect;
        //	Point send_point = select_the_rect(result_pic,Rect_vector,armour_area);
        Point send_point = select_the_rect(result_pic, Rect_vector, armour_area, select_rect);

        //	send_point = Point(send_point.x + Roi_rect.x, send_point.y + Roi_rect.y);

        rectangle(result_pic,
                  Rect(select_rect.x - Roi_rect.x, select_rect.y - Roi_rect.y, select_rect.width, select_rect.height),
                  Scalar(0, 255, 255), 2, 8);
        //绘制框图

//        circle(result_pic, send_point, 2, Scalar(255, 0, 255), 2, 8);
        cost_time = (get_sys_time() - cost_time) / getTickFrequency();

        //! 输出
//        display("display" , result_pic);
        cout << "fps:" << 1. / cost_time << endl;
        //index_cap.open(0);
        if (send_point == ERROR_POINT) {
            target_flags = false;
            not_find_count++;
            if (not_find_count > NO_FIND_COUNT) {
                setDataColor(0); //设置找不到
            }
            kf.ResetKalmanFilter(320, 240);
            {
                cout << "x:" << 0 << " y:" << 0 << endl;
                serialPosData(0, 0, distance_single, cost_time);
                setDataCmd(1);
            }
            test.push_back(Point(320, 240));
            int system_ret = system("clear");
            return;
        }

        //重置计数器
        setDataColor(BEAT_COLOR);
        not_find_count = 0;


//        cout << "distance is : " << distance_single << endl;
        //调用PNP


        distance_single = pnp_Get_Distance_armour(select_rect);
        target_flags = true;
        cout << "distance : " << distance_single << endl; //距离
//        std::cout << "x :" << send_point.x - 320 << "y:" << send_point.y - 240 << std::endl;
        Point armour_target = Point(send_point.x - 320, send_point.y - 240);
        cout << "armour_target.x:" << armour_target.x << "armour_target.y:" << armour_target.y << endl;

        //反 卡尔曼滤波 预测
        Point kalman_point = kf.run(armour_target.x, armour_target.y);
        Point anti_kalman_point = kf.Anti_KalmanFilter(armour_target, kalman_point, 0.9); //0.9为参数 可调

        //转换点坐标至屏幕左上角（ROI区域）
        Point2f target_point = Point2f(armour_target.x + 320, armour_target.y + 240);
        Point2f target_point_kalman = Point2f(anti_kalman_point.x + 320, anti_kalman_point.y + 240);

        //描绘装甲板中心
        circle(src_image, target_point, 3, Scalar(0, 255, 0), 2, 8);//current point with green
        //描绘卡尔曼滤波预测点
        circle(src_image, target_point_kalman, 3, Scalar(255, 0, 0), 2, 8);//predicted position with blue *
        test.push_back(send_point);

        //传出距离数据
        data_send.z = distance_single * 10;

        prev_distance = distance_single;
        prev_point_fire = send_point;

        //验证角度
        angle_vector = Calculate_angle(Point(50, 0), Point(send_point.x - 320, send_point.y - 240),
                                       distance_single * 10);

        ///解除注释以使用左右灯条角度预测
//        vector<double> angle_vector_predict_left = Calculate_angle(Point(50, 0),
//                                                                   Point(send_point.x - select_rect.width / 2.0 - 320,
//                                                                         send_point.x + select_rect.width / 2.0 - 320),
//                                                                   distance_single * 10);
//        vector<double> angle_vector_predict_right = Calculate_angle(Point(40, 0),
//                                                                    Point(send_point.x + select_rect.width / 2.0 - 320, 0),
//                                                                    distance_single * 10);

//        cout << "angle_vector[1] : " << angle_vector[1] << " angle_vector[2]:" << angle_vector[2] << endl;

        if (angle_vector[0] > 30.0) {
//            cout << "yaw:" << angle_vector[0] << endl;
            angle_vector[0] = 4.0;
            //cout << "enter "<< endl;
            //  cout << "> 40.0" << endl;
        } else if (angle_vector[0] < -6) {
//            cout << "yaw :" << angle_vector[0] << endl;
            angle_vector[0] = -2.0;
        }

        data_send.x_c = angle_vector[0] * 1000;
        data_send.y_c = angle_vector[1] * 1000;

        serialPosData(&data_send);
        ///解除注释以使用左右灯条角度预测
//        serialPosData(angle_vector[0] * 1000, angle_vector[1] * 1000, angle_vector_predict_left[0] * 1000, angle_vector_predict_right[0] * 1000);

        setDataCmd(1);

//        vector<double> kalman_angle_vector = Calculate_angle(Point(0, 0), Point(target_point_kalman.x - 320,
//                                                                                target_point_kalman.y - 240),
//                                                                                distance_single);
//        cout << "x_angle: " << angle_vector[0] << endl;
//        cout << "PH_angle: " << angle_vector[1] << endl;
//        cout << "kalman_angle: " << kalman_angle_vector[0] << endl;

        //! DEBUG 图像
        display("display",src_image);
        int system_ret = system("clear");
    }

}
