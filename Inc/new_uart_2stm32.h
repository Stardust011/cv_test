#ifndef _NEW_UART_2STM32
#define _NEW_UART_2STM32
/*<inttypes.h>:Í·ÎÄŒþÊÇÃèÊöÊýŸÝÀàÐÍµÄ×ÖœÚÊý
#pragma pack(8):Îª 8 ×ÖœÚ¶ÔÆë£¬ÔÚgccÖÐÓÐÓÃµœ
*/
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
    /*
    ·¢ËÍÏÂÎ»»úÖ¡Í·ÊýŸÝ
    */
    uint8_t Head;        //ÊýŸÝÖ¡Í·
    uint8_t CmdID;    //ÃüÁîID
    uint16_t Datalength;//ÊýŸÝ³€¶È

}__attribute__((packed, aligned(1))) Frame_hearder;

typedef struct {
    /*
    ×Ô¶¯Žò»÷ÊýŸÝÄ£Êœ
    */
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t Time_Interval;        //Ê¶±ðÊ±ŒäŒäžô
    uint8_t Goal_State;            //Ä¿±ê×ŽÌ¬

}__attribute__((packed, aligned(1))) Armour_MOD;
typedef struct {
    /*
        ÊÕµœ»ØžŽÊýŸÝÄ£Êœ
    */
    uint8_t Mod_set_over;    //»ØžŽÉèÖÃ³É¹ŠµÄÄ£Êœ

}__attribute__((packed, aligned(1))) Reply_MOD;

typedef struct {
    /*
        ÊÕµœŽó·ûÊýŸÝÄ£Êœ
    */
    float x;
    float y;
    float z;
    uint8_t Target_state;    //Ä¿±ê×ŽÌ¬


}__attribute__((packed, aligned(1))) Power_MOD;
typedef struct {
    /*
        ŽíÎó·ŽÀ¡
    */
    uint8_t Target_state;    //Ä¿±ê×ŽÌ¬£¬ÄÜŽò²»ÄÜŽò;

}__attribute__((packed, aligned(1))) Error_MOD;
//ÊÓŸõÊýŸÝœá¹¹
typedef struct {
    Frame_hearder FH_data;    //Ö¡Í·ÊýŸÝ
    union    //ÁªºÏ
    {
        Armour_MOD Ar_data;    //×Ô¶¯Žò»÷ÊýŸÝÄ£Êœ
        Reply_MOD Re_data;    //»ØžŽÊýŸÝ
        Power_MOD PW_data;    //Ð¡·ûÊýŸÝÄ£Êœ
        Error_MOD Err_data;    //ŽíÎó·ŽÀ¡Ä£Êœ
    } Data;
    uint16_t CRC16;         //Ö®Ç°ËùÓÐÊýŸÝCRCÐ£Ñé   ×¢ÒâŽËÊýŸÝºÍÖ®Ç°µÄÊýŸÝ¿ÉÄÜ²»Á¬Ðø£¬ËùÒÔ²»ÒªÖ±œÓÊ¹ÓÃ£¬ÈôÐèÒªÖ±œÓÊ¹ÓÃ£¬±ØÐëÔÚŽËž³Öµ
}__attribute__((packed, aligned(1))) Date_message;  //ÊýŸÝÖ¡

//·¢ËÍŽó·ûÊýŸÝ¶Î
//void send_message_PW(float xdata,float ydata,float zdata,uint8_t Cmdata);
//·¢ËÍ×°Œ×°åÊýŸÝ¶Î
void send_message_AR(float xdata, float ydata, float zdata, float tdata, uint8_t Cmdata);
//·¢ËÍŽíÎó·ŽÀ¡ÊýŸÝ¶Î
//void send_message_ER(uint8_t Cmdata);
//·¢ËÍÐ£ÑéÊýŸÝ¶Î
//void send_message_VR( uint8_t Cmdata);
#endif
