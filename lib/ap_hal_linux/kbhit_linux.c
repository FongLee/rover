#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <unistd.h>

static struct termios ori_attr, cur_attr;

/**
 * undo tty
 * @return  0: success; -1: error
 */
int tty_reset(void)
{

	if (tcsetattr(STDIN_FILENO, TCSANOW, &ori_attr) != 0)
		return -1;

	return 0;
}

/**
 * set attribution of tty
 * @return  0: success; -1: error
 */
int tty_set(void)
{
	if ( tcgetattr(STDIN_FILENO, &ori_attr) )
			return -1;
	memset(&cur_attr, 0, sizeof(cur_attr));
	memcpy(&cur_attr, &ori_attr, sizeof(cur_attr));
	cur_attr.c_lflag &= ~ICANON; //noncanonical mode
	cur_attr.c_lflag &= ~ECHO; //no echo
	cur_attr.c_cc[VMIN] = 1;
	cur_attr.c_cc[VTIME] = 0;

	if (tcsetattr(STDIN_FILENO, TCSANOW, &cur_attr) != 0)
			return -1;

	return 0;
}

/**
 * realization of kbhit in linux
 * @return  0: key down; -1: no kye down
 */
int kbhit(void)
{

	fd_set rfds;
	struct timeval tv;
	int retval;

	/* Watch stdin (fd 0) to see when it has input. */
	FD_ZERO(&rfds);
	FD_SET(0, &rfds);
	/* Wait up to five seconds. */
	tv.tv_sec  = 0;
	tv.tv_usec = 0;

	retval = select(1, &rfds, NULL, NULL, &tv);
	/* Don't rely on the value of tv now! */
	if (retval == -1)
	{
		return -1;
	}
	else if (retval)
		return 0;
	/* FD_ISSET(0, &rfds) will be true. */
	else
		return -1;

}

