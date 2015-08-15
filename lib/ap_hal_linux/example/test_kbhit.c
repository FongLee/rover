#include <stdio.h>

#include "kbhit_linux.h"

int main()
{
	int tty_set_flag;
	tty_set_flag = tty_set();
	//tty_set();
	while(1)
	{
		if( kbhit() )
		{
			const int key = getchar();
			if (key == '\n')
			{
				printf("  pressed enter \n");
			}
			else
				printf("%c pressed\n", key);
			if(key == 'q')
					break;
		}
	}

	if(tty_set_flag == 0)
	   tty_reset();
	return 0;
}
