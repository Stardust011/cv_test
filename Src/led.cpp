#include "led.h"

unsigned char off[] = "0";
unsigned char on[] = "1";

int gpio158_open() {
    int file_flags = open("/sys/class/gpio/gpio298/value", O_RDWR);
    if (file_flags < 0) {
        cout << "gpio open faild" << endl;
        return 0;
    }
    return file_flags;
}

void gpio158_write_on(int file_describe) {
    int ret = write(file_describe, on, sizeof(on));
    if (ret < 0) {
        cout << "write data failed" << endl;
    }
    return;
}

void gpio158_write_off(int file_describe) {
    int ret = write(file_describe, off, sizeof(off));
    if (ret < 0) {
        cout << "write off failed" << endl;
    }
    return;
}


void gpio158_close(int file_describe) {
    close(file_describe);
}


void led_red(int file_describe) {
    gpio158_write_on(file_describe);
    usleep(1000 * 100);
    gpio158_write_off(file_describe);
    usleep(1000 * 100);
}


void led_blue(int file_describe) {
    gpio158_write_on(file_describe);
    usleep(1000 * 1000);
    gpio158_write_off(file_describe);
    usleep(1000 * 1000);
}


