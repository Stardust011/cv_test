#ifndef _NEW_UART_THREAD_INIT_H
#define _NEW_UART_THREAD_INIT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <sys/ipc.h>   
#include <sys/msg.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>

#include "new_uart_2stm32.h"

#define BUFF_REPEAT 1
#define BUFF_UNREPEAT 2
#define BUFF_CENTER 3
#define BUFF_UNCENTER 4 

#define BEGIN_LED 1
#define BLUE_LED  2
#define RED_LED 3
//²Ëµ¥×÷ÎªÑ¡Ïî
enum
{
	MOD_NULL = 0,
	MOD_RED ,	//ºì·½
	MOD_BLUE,	//À¶·½
	MOD_POWER_BIG,	//´ò´ó·ù
	MOD_POWER_SMALL,//Ð¡·û
	MOD_AUTO,		//×Ô¶¯´ò»÷
};
/*====send the mesage====*/
//void send_message_PW(float xdata, float ydata, float zdata, uint8_t Cmdata);
void send_message_AR(float xdata, float ydata, float zdata, float tdata, uint8_t Cmdata);
//void send_message_VR( uint8_t Cmdata);
//void send_message_ER(uint8_t Cmdata);
/*=====the thread====*/
void *thread_read(void *arg);
void *thread_write(void * arg);
//void *thread_video(void * arg);
//void *thread_led(void *arg);
/*===== uart =====*/
int INIT_UART();
int UART_WRITE_DATA(char *data,int data_sizeof);



#endif/*end the init*/

