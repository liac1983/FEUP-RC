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

int fd; // Variável global para o descritor da porta serial
unsigned char buf[BUF_SIZE]; // Buffer global para o quadro SET

volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
volatile int retransmissions = 0;
volatile int STOP = FALSE;

void memdump(void *addr, size_t bytes) {
    for (size_t i = 0; i < bytes; ++i)
        printf("%02x ", *((char*) addr + i));
}

// Função para enviar o quadro SET
void send_set_frame() {
    int bytes = write(fd, buf, 5);
    printf("%d bytes written ( ", bytes);
    
    printf(")\n");
}

// Alarm handler para retransmitir o quadro SET após o timeout
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);

    // Retransmitir o quadro SET
    if (retransmissions < MAX_RETRANSMISSIONS) {
        printf("Retransmitting SET frame...\n");
        
        send_set_frame();
        alarm(3);
        alarmEnabled = TRUE;
        retransmissions++;
    }
}

int main(int argc, char *argv[]) {
    const char *serialPortName = argv[1];

    if (argc < 2) {
        printf("Incorrect program usage\nUsage: %s <SerialPort>\n", argv[0]);
        exit(1);
    }

    // Open serial port device
    fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }
    // Save current port settings
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
    newtio.c_cc[VTIME] = 0; // Timeout de 0.3 segundos (3 * 0.1s)
    newtio.c_cc[VMIN] = 5 ;  // Não bloquear, retorna após timeout ou ao receber qualquer byte
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    // Inicializar o buffer SET
    buf[0] = FLAG;
    buf[1] = 0x03;
    buf[2] = SET;
    buf[3] = buf[1] ^ buf[2]; // BCC1
    buf[4] = FLAG;

    unsigned char expected[5] = {FLAG, 0x03, UA, 0x03 ^ UA, FLAG};

    // Instalar o handler do alarme
    (void)signal(SIGALRM, alarmHandler);

    // Enviar o primeiro quadro SET
    send_set_frame();

    // Iniciar o alarme para timeout
    alarm(TIMEOUT);
    alarmEnabled = TRUE;

    int bytes;

    while (retransmissions < MAX_RETRANSMISSIONS) {
        //printf("Waiting for UA frame...\n");

        // Leitura não bloqueante com timeout
        bytes = read(fd, buf, 5); // Tenta ler o UA frame
       
        if (bytes == 5) {
            if (memcmp(buf, expected, 5) == 0) {
                printf("UA frame received: ");
                memdump(buf, 5);
                printf("\n");
                alarm(0); // Desativa o alarme
                break;    // Sai do loop quando o UA for recebido
            } else {
                printf("Frame received but not UA, ignoring...\n");
            }
        } 

        /* if (bytes == 0 && alarmEnabled == FALSE) {
            send_set_frame();
            alarm(3);
            alarmEnabled = TRUE;
        } */

        // Se o alarme soar, a função alarmHandler fará a retransmissão
        if (alarmCount >= MAX_RETRANSMISSIONS) {
            printf("Maximum retransmissions reached.\n");
            break;
        }
    }

    // Restaurar as configurações antigas da porta serial
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 0;
}
