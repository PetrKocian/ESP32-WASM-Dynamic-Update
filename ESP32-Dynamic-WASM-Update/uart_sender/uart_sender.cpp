// hello.c
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "test_wasm.h"

unsigned char wasm_start_seq[] = {0x77, 0x61, 0x73, 0x6D}; // "wasm"

unsigned char US[] = {0x1F};
unsigned char STX[] = {0x02};
unsigned char ETX[] = {0x03};

int main() {
    int fd = open("/dev/ttyUSB0", O_RDWR);

    printf("uart_sender: SENDING DATA\r\n");
    write(fd, STX, 1);
    write(fd, wasm_start_seq, sizeof(wasm_start_seq));
    write(fd, US, 1);
    write(fd, wasm_test_file, sizeof(wasm_test_file));
    //write(fd, ETX, 1);
    printf("uart_sender: DATA SENT\r\n");

    return 0;
}
