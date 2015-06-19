
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "uart_driver.h"


//int fd_uart = -1;
int fd_gps = -1;
int fd_ultr = -1;

int uart_init(int *fd_uart, const char *serial,int baud)         //open uart and initialize the uart
{	
	//int BAUDRATE;
	
	struct termios oldtio,newtio;
	//printf("\n");   
	int uartbiit[50] = {B115200,B9600,B19200,B4800,B2400,B1200}; 
	//*fd_uart = open(serial, O_RDWR | O_NOCTTY );
	*fd_uart = open(serial, O_RDWR);
	if( *fd_uart <0)
	{
		//perror("error: can't open serial port!");
		//exit(1);
		return -1;
	}
	//else 
		//printf("Serial port open successed!!!\n");
	//fprintf(stdout, "\nSerial port open successed!!!\n");
	tcgetattr(*fd_uart,&oldtio); //save current serial port settings
	bzero(&newtio, sizeof(newtio)); // clear struct for new port settings
	//BAUDRATE = B115200;
	cfsetispeed(&newtio,uartbiit[baud]);
	cfsetospeed(&newtio,uartbiit[baud]);
	// newtio.c_cflag |= CS8 | CLOCAL | CREAD;
	// newtio.c_iflag = 0;
	// newtio.c_oflag = 0;
	// newtio.c_lflag = 0;

	newtio.c_lflag &= ~(ECHO |ICANON | IEXTEN |ISIG);//no echo,non-standard input,no expansion character processing,no response to signal
	newtio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);//produce no sigint when gets BREAK,no change CR into NL,close parity，keep eight bits,close output control
	newtio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);//no mask date of character，disable parity,use one stop bit
	newtio.c_cflag |= CS8 | CLOCAL | CREAD;//a character contained 8 bits,ignore the modem status line,enable the receiver
	newtio.c_oflag &= ~(OPOST);//do not perform output processing

	newtio.c_cc[VINTR] = 0; /* Ctrl-c*/ 
	newtio.c_cc[VQUIT] = 0; /* Ctrl-\ */
	newtio.c_cc[VERASE] = 0; /* del */
	newtio.c_cc[VKILL] = 0; /* @ */
	newtio.c_cc[VEOF] = 0; /* Ctrl-d */
	newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
	newtio.c_cc[VMIN] = 0; /* blocking read until 1 characterarrives */
	newtio.c_cc[VSWTC] = 0; /* '\0' */
	newtio.c_cc[VSTART] = 0; /* Ctrl-q */
	newtio.c_cc[VSTOP] = 0; /* Ctrl-s */
	newtio.c_cc[VSUSP] = 0; /* Ctrl-z */
	newtio.c_cc[VEOL] = 0; /* '\0' */
	newtio.c_cc[VREPRINT] = 0; /* Ctrl-r */
	newtio.c_cc[VDISCARD] = 0; /* Ctrl-u */
	newtio.c_cc[VWERASE] = 0; /* Ctrl-w */
	newtio.c_cc[VLNEXT] = 0; /* Ctrl-v */
	newtio.c_cc[VEOL2] = 0; /* '\0' */
	tcflush(*fd_uart, TCIOFLUSH);
	tcsetattr(*fd_uart,TCSANOW,&newtio);
	
	//fprintf(stdout, "\nuart config successed!!!\n");
	return 0;	
}

/*****************************************************************************
 Prototype    : uart_close
 Description  : close uart4 
 Input        : None
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2014/11/7
    Author       : xuanfeng
    Modification : Created function

*****************************************************************************/
void uart_close(int fd_uart)
{
	
	if (fd_uart != -1)
	{
		close(fd_uart);
		fd_uart = -1;
	}	
	//return -1;	
}

int read_uart(int fd_uart, char *buf, unsigned int n)
{
	int len;
	len = read(fd_uart,buf,n);
	return len;
}

int write_uart(int fd_uart, char *buf,unsigned int n)
{
	int len;
	len=write(fd_uart,buf,n);
	return len;
}