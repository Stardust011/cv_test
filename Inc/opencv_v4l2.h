#ifndef _OPENCV_V4L2_H
#define _OPENCV_V4L2_H

#include <unistd.h>
#include <error.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace cv;
#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define IMAGEWIDTH 3264
#define IMAGEHEIGHT 2448

void VideoPlayer(Mat &image_out) ;

class V4L2Capture {
public:
	V4L2Capture(char *devName, int width, int height);
	virtual ~V4L2Capture();

	int openDevice();
	int closeDevice();
	int initDevice();
	int startCapture();
	int stopCapture();
	int freeBuffers();
	int getFrame(void **,size_t *);
	int backFrame();
	static void test();

private:
	int initBuffers();

	struct cam_buffer
	{
		void* start;
		unsigned int length;
	};
	char *devName;
	int capW;
	int capH;
	int fd_cam;
	cam_buffer *buffers;
	unsigned int n_buffers;
	int frameIndex;
};
V4L2Capture * set_v4l(char * video_path = "/dev/video0");
Mat VideoCapture_v4l(V4L2Capture * vcap);


#endif/*_OPENCV_V4L2_H*/
