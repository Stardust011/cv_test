#ifndef _NEW_UART_2STM32
#define _NEW_UART_2STM32
/*<inttypes.h>:
#pragma pack(8):
*/
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <cstdlib>

typedef struct {

    uint8_t Head;
    uint8_t CmdID;
    uint16_t Datalength;

}__attribute__((packed, aligned(1))) Frame_hearder;

typedef struct {

    int32_t x;
    int32_t y;
    int32_t z;
    int32_t Time_Interval;
    uint8_t Goal_State;

}__attribute__((packed, aligned(1))) Armour_MOD;
typedef struct {

    uint8_t Mod_set_over;

}__attribute__((packed, aligned(1))) Reply_MOD;

typedef struct {

    float x;
    float y;
    float z;
    uint8_t Target_state;


}__attribute__((packed, aligned(1))) Power_MOD;
typedef struct {

    uint8_t Target_state;

}__attribute__((packed, aligned(1))) Error_MOD;
//ÊÓŸõÊýŸÝœá¹¹
typedef struct {
    Frame_hearder FH_data;
    union
    {
        Armour_MOD Ar_data;
        Reply_MOD Re_data;
        Power_MOD PW_data;
        Error_MOD Err_data;
    } Data;
    uint16_t CRC16;
}__attribute__((packed, aligned(1))) Date_message;

void send_message_AR(float xdata, float ydata, float zdata, float tdata, uint8_t Cmdata);
#endif
