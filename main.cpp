//
// Created by Stardust on 2023/3/25.
//

#include <iostream>
#include <string>
#include <WzSerialPortPlus.h>
#include "serial.h"

using namespace std;

//int main() {
//    WzSerialPortPlus serialPort;
//    /*serialPort.setReceiveCalback([&](char* data, int length){
//        printf("received: %s\n",data);
//
//        std::string responsePrefix = "received: ";
//        std::string response(data,length);
//        response = responsePrefix + response;
//
//        serialPort.send((char*)response.c_str(), response.length());
//    });*/
//    serialPort.open("/dev/ttyUSB0", 115200, 1, 8, 'n');
//    return 0;
//}
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
    int i;
    pos_data data1;
    data1.x = 1;
    data1.y = 6;
    data1.z = 4;
    data1.time_interval = 9;
    serialData1.data = data1;
    cout.setf(ios::hex);
    for(i=0;i<sizeof(serialData1);i++){
        printf("%#x ",((char*)&serialData1)[i]);
//        cout<<oct<<i<<dec<<i<<hex<<i<<endl;
    }
    serialPort.open("/dev/ttyUSB0", 115200, 1, 8, 'n');
    serialSend(data1.x,data1.y,data1.z,data1.time_interval);
    return 0;
}