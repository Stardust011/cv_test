//
// Created by Stardust on 2023/3/29.
//

#include "serial.h"

int MOD_B_R = 1;
bool READ_DATA;
bool Set_Mod = true;
int CMD_COLOR = 1;

WzSerialPortPlus serialPort;
serialData data1;

/*!
 * @brief 初始化串口
 */
bool serialInit() {
    // Init serial port
    bool ret = serialPort.open(SERIAL_PORT, SERIAL_BAUDRATE, 1, 8, 'n');
    return ret;
}

/*!
 * @brief CRC16校验
 * @param data_p
 * @param length
 * @return unsigned short crc16
 */
static unsigned short crc16(const unsigned char *data_p, unsigned char length) {
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--) {
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((unsigned short) (x << 12)) ^ ((unsigned short) (x << 5)) ^ ((unsigned short) x);
    }
    return crc;
}

/*!
 * @brief 发送数据
 * @param x
 * @param y
 * @param z
 * @param time_interval
 */
void serialPosData(int32_t x, int32_t y, int32_t z, int32_t time_interval) {
    data1.data.x = x;
    data1.data.y = y;
    data1.data.z = z;
    data1.data.time_interval = time_interval;
    union crc16 crc_data{};
    crc_data.crc = crc16((unsigned char *) &data1, sizeof(data1) - 4);
    data1.crc[0] = crc_data.crc_char[0];
    data1.crc[1] = crc_data.crc_char[1];
}

/*!
 * @brief 发送数据
 * @param pos_data data
 */
void serialPosData(pos_data *data) {
    data1.data.x = data->x;
    data1.data.y = data->y;
    data1.data.z = data->z;
    data1.data.time_interval = data->time_interval;
    union crc16 crc_data{};
    crc_data.crc = crc16((unsigned char *) &data1, sizeof(data1) - 4);
    data1.crc[0] = crc_data.crc_char[0];
    data1.crc[1] = crc_data.crc_char[1];
    serialPort.send((char *) &data1, sizeof(data1));
}

/*!
 * @brief 关闭串口
 */
void serialClose() {
    serialPort.close();
}

/*!
 * @brief 读取串口数据
 * @param char *data
 * @param int length
 *
 * 有一说一 我根本不知道为什么这里是这样的
 */
void *serialRead(void *pVoid) {
    for (;;) {
        serialPort.setReceiveCalback([&](char *data, int length) {
            printf("received: %s\n", data);
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

void *serialSend(void *pVoid) {
    for (;;) {
        serialPort.send((char *) &data1, sizeof(data1));
    }
}

void setDataCmd(int cmd) {
    data1.cmd = cmd;
}