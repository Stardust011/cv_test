//
// Created by Stardust on 2023/3/29.
//

#include "serial.h"

int MOD_B_R = 1;
bool READ_DATA;
bool Set_Mod = true;
int CMD_COLOR = 1;

WzSerialPortPlus serialPort;

void serialInit() {
    // Init serial port
    serialPort.open(SERIAL_PORT, SERIAL_BAUDRATE, 1, 8, 'n');
}

void serialSend(int32_t x, int32_t y, int32_t z, int32_t time_interval) {
    serialData data;
    data.data.x = x;
    data.data.y = y;
    data.data.z = z;
    data.data.time_interval = time_interval;
    serialPort.send((char *) &data, sizeof(data));
}

void serialClose() {
    serialPort.close();
}

void serialRead(){
    for(;;) {
        serialPort.setReceiveCalback([&](char* data, int length){
            printf("received: %s\n",data);
                if ((unsigned char) data[0] == 0xA5 && (unsigned char) data[1] == 0x05) {

                    if (data[4] == 0x01) {

                        MOD_B_R = MOD_RED;
                        READ_DATA = true;
                        Set_Mod = true;
                        CMD_COLOR = 1;
                        //bule_or_red = false;
                    } else if (data[4] == 0x02) {

                        MOD_B_R = MOD_BLUE;
                        READ_DATA = true;
                        Set_Mod = true;
                        CMD_COLOR = 2;
                        //bule_or_red = false;
                    } else if (data[4] == 0x04) {

                        //printf("enter power big .." );
                        MOD_B_R = MOD_POWER_BIG;

                        READ_DATA = true;
                        Set_Mod = true;
                    } else {
                        MOD_B_R = MOD_NULL;
                    }
            }
        });
    }
}

unsigned short CRC16(const unsigned char* data_p, unsigned char length){
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc;
}
