#ifndef KBHIT_LINUX_H
#define KBHIT_LINUX_H

/**
 * undo tty
 * @return  0: success; -1: error
 */
int tty_reset(void);

/**
 * set attribution of tty
 * @return  0: success; -1: error
 */
int tty_set(void);

/**
 * realization of kbhit in linux
 * @return  0: key down; -1: no kye down
 */
int kbhit(void);

#endif
