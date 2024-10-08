// Read from serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 5

#define FLAG 0x7E
#define A 0x03
#define C_SET 0x03

typedef enum {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP} State;

void memdump (void *addr, size_t bytes) {
    for (size_t i = 0; i < bytes; i++) {
        printf("%02x ", *((char*) addr + i));
    }
}

//volatile int STOP = FALSE;

int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];
    unsigned char byte;
    State state = START;  // Initialize state

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 1 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    unsigned char bcc1;

    while (state != STOP) {
        int res = read(fd, &byte, 1);  // Read a byte from the serial port
        if (res < 0) {
            perror("Error reading from serial port");
            return -1;
        }

        switch (state) {
            case START:
                if (byte == FLAG) {
                    printf("FLAG = 0x%02X\n", byte);  // Print FLAG
                    state = FLAG_RCV;
                }
                break;

            case FLAG_RCV:
                if (byte == A) {
                    printf("ADDRESS = 0x%02X\n", byte);  // Print ADDRESS
                    state = A_RCV;
                } else if (byte != FLAG) {
                    state = START;
                }
                break;

            case A_RCV:
                if (byte == C_SET) {
                    printf("CONTROL = 0x%02X\n", byte);  // Print CONTROL
                    state = C_RCV;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case C_RCV:
                bcc1 = A ^ C_SET;  // Calcutate BCC1
                if (byte == bcc1) {
                    printf("BCC1 = 0x%02X\n", byte);  // Print BCC1
                    state = BCC_OK;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case BCC_OK:
                if (byte == FLAG) {  // The last FLAG closes the frame correctly
                    printf("FLAG = 0x%02X\n", byte);  // Print seconde FLAG
                    state = STOP;
                } else {
                    state = START;
                }
                break;

            default:
                state = START;
                break;
        }
    }

    printf("Frame SET received correctly!\n");

    // Loop for input
    /*unsigned char buf[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char
    while (STOP == FALSE)
    {
        int bytes = read(fd, buf, BUF_SIZE);

        printf("Flag = 0x%02x\n", buf[0]);
        printf("Adress = 0x%02x\n", buf[1]);
        printf("Control = 0x%02x\n", buf[2]);
        printf("BCC1 = 0x%02x\n", buf[3]);
        printf("Flag #2 = 0x%02x\n\n", buf[4]);

        if(buf[0] == 0x7E && buf[1] == 0x03 && buf[2] == 0x03 && buf[3] == 0x00 && buf[4] == 0x7E)
            STOP = TRUE;

    }*/
    sleep(1);

    // Answer to Receiver

    unsigned char buf_answer[BUF_SIZE] = {0};

    unsigned char flag = 0x7E;
    unsigned char control = 0x07;
    unsigned char address = 0x03;
    unsigned char bcc1_1 = control ^address;

    buf_answer[0] = flag;
    buf_answer[1] = address;
    buf_answer[2] = control;
    buf_answer[3] = bcc1_1;
    buf_answer[4] = flag;

    int bytes = write(fd, buf_answer, BUF_SIZE);
    printf("%d bytes written to answer\n", bytes);

    memdump(buf_answer, 5);
    printf("\n");

    /*while (STOP == FALSE)
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, BUF_SIZE);
        buf[bytes] = '\0'; // Set end of string to '\0', so we can printf

        printf(":%s:%d\n", buf, bytes);
        if (buf[0] == 'z')
            STOP = TRUE;
    }*/

    // The while() cycle should be changed in order to respect the specifications
    // of the protocol indicated in the Lab guide

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
