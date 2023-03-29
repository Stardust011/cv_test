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

struct pos_data {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t time_interval;
    pos_data() {
        x = 0;
        y = 0;
        z = 0;
        time_interval = 0;
    }
};

struct serialData {
    unsigned char head[2] ;
    unsigned char cmd;
    unsigned char length;
    pos_data data;
    unsigned char crc[2]{};
    serialData() {
        head[0] = 0xA5;
        head[1] = 0xFF;
        cmd = 0x01;
        length = 0x18;
        pos_data data_init;
        crc[0] = 0x00;
        crc[1] = 0x00;
    }
};

class serial {
};

void serialInit();
void serialSend(int32_t x, int32_t y, int32_t z, int32_t time_interval);
void serialClose();
void serialRead();
unsigned short CRC16(const unsigned char* data_p, unsigned char length);

#endif //CV_TEST_SERIAL_H
