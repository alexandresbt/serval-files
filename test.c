#include <stdio.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>

void hex2str(char *input, char *output, int len){

	char hex_number[2];
	long number;

	for(int i=0;i<len; i+=2) {
		memcpy(hex_number, &input[i], 2);
		number = strtol(hex_number, 0, 16);
		output[i/2] = toascii(number);
	}
	output[(len/2) +1] = '\0';
}

void str2hex(char *input, char *output, int len)
{
    for(int i=0; i<len; i++)
        sprintf((char*)(output+(2*i)),"%02X", input[i]);
}

int set_interface_attribs (int fd, int speed, int parity) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
		fprintf(stderr, "error %d from tcgetattr\n", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
		fprintf(stderr, "error %d from tcsetattr\n", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block) {
	struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
		fprintf(stderr, "error %d from tggetattr\n", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
		 fprintf(stderr, "error %d setting term attributes\n", errno);
}

int main() {
	char *portname = "/dev/ttyUSB0";
	char *portname2 = "/dev/ttyUSB1";

	// connection opening
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	int fd2 = open (portname2, O_RDWR | O_NOCTTY | O_SYNC);

	if (fd < 0) {
		fprintf(stderr, "error %d opening %s: %s\n", errno, portname, strerror (errno));
        	return 0;}
	if (fd2 < 0) {
		fprintf(stderr, "error %d opening %s: %s\n", errno, portname2, strerror (errno));
        	return 0;}

	// speed configuration
	set_interface_attribs (fd, B57600, 0);  
	set_blocking (fd, 0);               
	set_interface_attribs (fd2, B57600, 0);  
	set_blocking (fd2, 0);               
	


	/* connection opened, starting tests */
    	char useless_buff_1[100];
    	char useless_buff_2[100];
	write (fd, "\r\n", 2);
	write (fd2, "\r\n", 2);
	usleep ((11 + 25) * 2 * 100); 
	read (fd, useless_buff_1, sizeof useless_buff_1); 
	read (fd2, useless_buff_2, sizeof useless_buff_2); 

	// requested configuration of radio
	write (fd, "mac pause\r\n", 11);
	write (fd2, "mac pause\r\n", 11);
	usleep ((11 + 25) * 2 * 100); */            
         
	// clearing reception buffer             
	char buf [100];
	read (fd, buf, sizeof buf); 
	printf("response mac pause from USB0: %s\n", buf);

	char buf2 [100];
	read (fd2, buf2, sizeof buf2); 
	printf("response mac pause from USB1: %s\n", buf2);
	// connection working & cleared



	// Tests part

	int size;
	write (fd, "radio rx 0\r\n", 14);
	char buf3 [100];
	read (fd, buf3, sizeof buf3); 
	printf("reception state: %s\n", buf3);

	int size_tested = 24;
	char msg[size_tested + 9 + 4];

	strcpy(msg, "radio tx ");

	char test[13];
	char hex[24];
	strcpy(test, "Hello World!");
	printf("message : Hello World!\n"); 
	str2hex(test, hex, 12);
	strcat(msg, hex);
	strcat(msg, "\r\n");
	printf("command sent (after concatenation): %s\n", msg);          
        write (fd2, msg, (size_tested + 9 + 4));
	sleep(2);
	char buf4 [size_tested + 9 + 4];
	size = read (fd, buf4, sizeof buf4);
	printf("message received after transmission: %s\n", buf4);

	int msg_size = 24;
	char msg_reception_hex[msg_size];
	char msg_reception_str[(msg_size/2) + 1];
	char sub[11];
	memcpy(sub, &buf4[0], 10);
	//printf("sub :%st\nreception start :%st\n", sub, msg_reception_start);
	//printf("comparaison: %d\n", strcmp(sub, msg_reception_start));
	if(strcmp(sub, "radio_rx  ")==0) {
		printf("begenning of reception message detected\n");
		//printf("message: %s\n", buf4);
		char sub2[3];
		memcpy(sub2, &buf4[size_tested + 9 + 4 - 2], 1);
		//printf("end of message: %s\n", sub2);
		if(strcmp(sub2, "\n")==0) {
			printf("end of reception message detected\n");

			memcpy(msg_reception_hex, &buf4[10], msg_size);
			//printf("msg : %s\nsize: %d\n", msg_reception_hex, sizeof msg_reception_hex);
			hex2str(msg_reception_hex, msg_reception_str, msg_size);
			printf("msg received: %s\n", msg_reception_str);
		}
			
	}
	return 1;
}
