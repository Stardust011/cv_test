//
// Created by Stardust on 2023/3/25.
//

#include <iostream>
#include <string>
#include <WzSerialPortPlus.h>

int main() {
    WzSerialPortPlus serialPort;
    serialPort.setReceiveCalback([&](char* data, int length){
        printf("received: %s\n",data);

        std::string responsePrefix = "received: ";
        std::string response(data,length);
        response = responsePrefix + response;

        serialPort.send((char*)response.c_str(), response.length());
    });
    if(serialPort.open("/dev/ttyS1", 9600, 1, 8, 'n'))
    {
        getchar();
        serialPort.close();
    }

    return 0;
}