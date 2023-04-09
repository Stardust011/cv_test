//
// Created by Stardust on 2023/3/23.
//

#ifndef CV_TEST_CONFIG_H
#define CV_TEST_CONFIG_H


//#define WORK_WITH_WINDOWS 1
#define WORK_WITH_LINUX 1

#define IMAGE_WIDTH 3264
#define IMAGE_HEIGHT 2448
#define FPS 30
#define VIDEO "/dev/video1"
#define SERIAL_PORT "/dev/ttyUSB0"
#define SERIAL_BAUDRATE 115200

//id置零计数
#define NO_FIND_COUNT 80

//装甲板参数 (m)
#define a_height 0.055
#define a_width 0.140

//大符参数 (m)
#define b_height 0.2
#define b_width 0.3

// 1: red, 2: blue
#define BEAT_COLOR 1

#endif //CV_TEST_CONFIG_H
