//
// Created by Stardust on 2023/3/25.
//

#include <iostream>
#include <string>
#include <WzSerialPortPlus.h>
#include "serial.h"

using namespace std;

unsigned short crc16(const unsigned char* data_p, unsigned char length){
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc;
}

int main() {
    WzSerialPortPlus serialPort;
    serialData serialData1;
    serialPort.open("/dev/ttyUSB0", 115200, 1, 8, 'n');

    int i;
    cout.setf(ios::hex);
    union crc16 crc_data{};

    serialData1 = serialPosData(1, 2, 3, 4);
    crc_data.crc = crc16((unsigned char*)&serialData1, sizeof(serialData1)-4);
    serialData1.crc[0] = crc_data.crc_char[0];
    serialData1.crc[1] = crc_data.crc_char[1];
    serialPort.send((char*)&serialData1, sizeof(serialData1));

    for(i=0;i<sizeof(serialData1);i++){
        printf("%#x ",((char*)&serialData1)[i]);
//        cout<<oct<<i<<dec<<i<<hex<<i<<endl;
    }
    serialPort.open("/dev/ttyUSB0", 115200, 1, 8, 'n');
    return 0;
}