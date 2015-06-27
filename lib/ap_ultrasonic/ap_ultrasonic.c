#include "ap_ultrasonic.h"

#define UART_DEVICE "/dev/ttySAC2"



int ultrasonic_init()
{
	if (uart_init(&fd_ultr, UART_DEVICE,1) < 0)
	{
		fprintf(stderr,"Open Uart ERR:%s\n",strerror(errno));
		return -1;
	}
	// fp_dis=fopen("distance.txt","wb");
	return 0;

}

int ultrasonic_read(float  *distance)
{


		int len;
		char temp[2];
		char device_addr=0xe8;
		char register_addr=0x02;
		char data=0xbc;
		//distance = 0.0;
		int res = 0;
		res = write_uart(fd_ultr, &device_addr,1);

		if (res < 0)
		{
		
#ifdef ULTRA_DEBUG		
			fprintf(stderr, "write_uart failed:%s\n", strerror(errno));
#endif
			return -1;			
		}

		usleep(30);
		res = write_uart(fd_ultr, &register_addr,1);
		if (res < 0)
		{
		
#ifdef ULTRA_DEBUG		
			fprintf(stderr, "write_uart failed:%s\n", strerror(errno));
#endif
			return -1;			
		}
		usleep(30);
		res = write_uart(fd_ultr, &data,1);
		if (res < 0)
		{
#ifdef ULTRA_DEBUG		
			fprintf(stderr, "write_uart failed:%s\n", strerror(errno));
#endif
			return -1;			
		}
		//usleep(10);
		len=read_uart(fd_ultr, &temp,2);//从串口读
		if (len < 0)
		{
		
#ifdef ULTRA_DEBUG		
			fprintf(stderr, "write_uart failed:%s\n", strerror(errno));
#endif
			return -1;			
		}

		//printf("%x\n",TH);
#ifdef ULTRA_DEBUG
		fprintf(stdout, "ULTRA temp[0] is %x\n",temp[0]);
		fprintf(stdout, "ULTRA temp[1] is %x\n",temp[1]);
#endif
		//printf("%x\n",temp[0]);
		//printf("%x\n",temp[1]);
		//len = n;
		//distance=(float)(TH*256+TL);
		*distance=(float)(temp[0]*256+temp[1]);
		return 0;
		//printf("%f\n",distance);



}


int ultrasonic_close()
{
		uart_close(&fd_ultr);
		//fclose(fp_dis);
}
