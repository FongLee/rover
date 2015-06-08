#include "ap_ultrasonic.h"
#define UART_DEVICE "/dev/ttySAC2"



int ultrasonic_init()
{
	if (uart_init(UART_DEVICE,1) != 0)
	{
		fprintf(stderr,"Open Uart ERR:%s\n",strerror(errno));
		return -1;
	}
	// fp_dis=fopen("distance.txt","wb");
	return 0;

}

int ultrasonic_read()
{


		int len;
		char temp[2];
		char device_addr=0xe8;
		char register_addr=0x02;
		char data=0xbc;
		distance = 0.0;
		write_uart(&device_addr,1);
		usleep(30);
		write_uart(&register_addr,1);
		usleep(30);
		write_uart(&data,1);
		//usleep(10);
		len=read_uart(&temp,2);//从串口读
		
		//printf("%x\n",TH);
		#ifdef ULTRA_DEBUG
		fprintf(stdout, "ULTRA temp[0] is %x\n",temp[0]);
		fprintf(stdout, "ULTRA temp[1] is %x\n",temp[1]);
		#endif
		//printf("%x\n",temp[0]);
		//printf("%x\n",temp[1]);
		//len = n;
		//distance=(float)(TH*256+TL);
		distance=(float)(temp[0]*256+temp[1]);
		//printf("%f\n",distance);
		
			
		
}


int ultrasonic_close()
{
		uart_close();
		//fclose(fp_dis);
}