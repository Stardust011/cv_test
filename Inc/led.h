#ifndef _LED_H
#define _LED_H

#include <cstdio>
#include <unistd.h>                   //·ûºÅ³£Á¿£¬ÔÚVC»·Ÿ³ÏÂ²»°üº¬Ëü¿ÉÄÜ»á±šŽí
#include <sys/ioctl.h>                //GPIO Control
#include <sys/stat.h>                 //»ñÈ¡ÎÄŒþÊôÐÔ
#include <linux/fs.h>                 //ÓëÉè±žÇý¶¯³ÌÐòÓÐ¹Ø
#include <fcntl.h>                    //žÄ±äÒÑŽò¿ªµÄÎÄŒþÐÔÖÊ
#include <cstring>
#include <termios.h>
#include <cerrno>
#include <iostream>
//#include "new_uart_thread_init.h"
#include "serial.h"

using namespace std;

extern int CMD_COLOR;

int gpio158_open();

void gpio158_write_on(int file_describe);

void gpio158_write_off(int file_describe);

void gpio158_close(int file_describe);

void led_red(int file_describe);

void led_blue(int file_describe);
//static void * led_on_thread(void * arg);

#endif
