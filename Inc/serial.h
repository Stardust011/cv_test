//
// Created by Stardust on 2023/3/29.
//

#ifndef CV_TEST_SERIAL_H
#define CV_TEST_SERIAL_H

#include <WzSerialPortPlus.h>
#include "config.h"

enum {
    MOD_NULL = 0,
    MOD_RED,
    MOD_BLUE,
    MOD_POWER_BIG,
    MOD_POWER_SMALL,
    MOD_AUTO,
};

enum COLOR{
    NONE = 0,
    RED,
    BLUE,
};

union crc16 {
    unsigned short crc;
    unsigned char crc_char[2];
};

struct cal_data{
    double x;
    double y;
    double z;
    double yaw;
    double pitch;
};

struct pos_data {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t time_interval;
    int32_t x_c;
    int32_t y_c;
    pos_data() {
        x = 0;
        y = 0;
        z = 0;
        time_interval = 0;
        x_c = 0;
        y_c = 0;
    }
};

struct serialData {
    unsigned char head{0xAA};
    unsigned char id{0x01};
    unsigned char cmd{0x01};
    unsigned char length{0x18};
    pos_data data;
    unsigned char crc[2]{0x00, 0x00};
    unsigned char end[2]{0x0D, 0x0A};
};

class serial {
};

bool serialInit();
serialData serialPosData(int32_t x, int32_t y, int32_t z, int32_t time_interval);
serialData serialPosData(pos_data* data);
void setDataCmd(int cmd);
void serialClose();
void * serialRead(void *pVoid);
void * serialSend(void *pVoid);
void setDataColor(int color);

#endif //CV_TEST_SERIAL_H
