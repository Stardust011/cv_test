#include "iostream"
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main_test() {
    Mat img = imread(R"(C:\Users\Stardust\Pictures\Saved Pictures\PIXIV\94905891_p0.jpg)");
    if (img.empty()) {
        cout << "Error" << endl;
        return -1;
    }
    imshow("Pic", img);
    waitKey();
    return 0;
}