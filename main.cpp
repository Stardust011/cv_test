//
// Created by Stardust on 2023/3/25.
//

#include <iostream>
#include <string>
#include <WzSerialPortPlus.h>
#include "serial.h"

using namespace std;

//unsigned short crc16(const unsigned char* data_p, unsigned char length){
//    unsigned char x;
//    unsigned short crc = 0xFFFF;
//
//    while (length--){
//        x = crc >> 8 ^ *data_p++;
//        x ^= x>>4;
//        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
//    }
//    return crc;
//}

int main() {
    WzSerialPortPlus serialPort;
    serialData serialData1;
    int i;
    pos_data data1;
    data1.x = 15565656;
    data1.y = 698;
    data1.z = 4;
    data1.time_interval = 9;
    serialData1.data = data1;
    cout.setf(ios::hex);
    union crc16 crc_data{};
    serialData1.crc[0] = crc_data.crc_char[0];
    serialData1.crc[1] = crc_data.crc_char[1];
    serialPosData(&data1);

    for(i=0;i<sizeof(serialData1);i++){
        printf("%#x ",((char*)&serialData1)[i]);
//        cout<<oct<<i<<dec<<i<<hex<<i<<endl;
    }
    serialPort.open("/dev/ttyUSB0", 115200, 1, 8, 'n');
    serialPosData(data1.x, data1.y, data1.z, data1.time_interval);
    return 0;
}