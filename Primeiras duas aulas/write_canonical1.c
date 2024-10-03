#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 256
#define MAX_RETRANSMISSIONS 3
#define TIMEOUT 3 // Timeout in seconds

#define FLAG 0x7E
#define SET 0x03
#define UA 0x07

volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
volatile int STOP = FALSE;

void memdump(void *addr, size_t bytes) {
    for (size_t i = 0; i < bytes; ++i)
        printf("%02x ", *((char*) addr + i));
}

// Alarm handler for timeout
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

int main(int argc, char *argv[]) {
    const char *serialPortName = argv[1];

    if (argc < 2) {
        printf("Incorrect program usage\nUsage: %s <SerialPort>\n", argv[0]);
        exit(1);
    }

    // Open serial port device
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio, newtio;
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    // Configure the new serial port settings
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    // Install alarm handler
    (void)signal(SIGALRM, alarmHandler);

    unsigned char buf[BUF_SIZE] = {0};
    buf[0] = FLAG;
    buf[1] = 0x03;
    buf[2] = SET;
    buf[3] = buf[1] ^ buf[2]; // BCC1
    buf[4] = FLAG;

    unsigned char expected[5] = {FLAG, 0x03, UA, 0x03 ^ UA, FLAG};
    int retransmissions = 0;

    while (retransmissions < MAX_RETRANSMISSIONS) {
        // Send the SET frame
        int bytes = write(fd, buf, 5);
        printf("%d bytes written ( ", bytes);
        memdump(buf, 5);
        printf(")\n");

        // Start the alarm for timeout
        alarm(TIMEOUT);
        alarmEnabled = TRUE;

        printf("Waiting for UA frame...\n");
        bytes = read(fd, buf, 5); // Blocking read for UA frame

        // Check if the UA frame is received
        if (memcmp(buf, expected, 5) == 0) {
            printf("UA frame received: ");
            memdump(buf, 5);
            printf("\nExpected frame: ");
            memdump(expected, 5);
            printf("\n");
            alarm(0); // Disable alarm
            break;    // Exit loop when UA is received
        } else {
            printf("UA frame not received, retransmitting...\n");
            retransmissions++;
        }

        if (alarmCount >= MAX_RETRANSMISSIONS) {
            printf("Maximum retransmissions reached.\n");
            break;
        }
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 0;
}

