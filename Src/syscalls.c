/* Support files for GNU libc.  Files in the system namespace go here.
   Files in the C namespace (ie those that do not start with an
   underscore) go in .c.  */

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include "stm32f1xx_hal.h"

/* Variables */
//#undef errno
extern int errno;

extern UART_HandleTypeDef huart1;

char *__env[1] = { 0 };
char **environ = __env;


/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	(void) pid;
	(void) sig;

	errno = EINVAL;
	return -1;
}

void _exit (int status)
{
	_kill(status, -1);
	while (1) {}		/* Make sure we hang here */
}

int _read(int file, char *ptr, int len)
{
	(void) file;
	(void) ptr;
	(void) len;

	return -1;
}

int _write(int file, char *ptr, int len)
{
	/* stdout - write to serial point */
	if (file == 1 && len > 0) {
		if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF) != HAL_OK)
			return -1;
		return 0;
	}

	return -1;
}

caddr_t _sbrk(int incr)
{
	register char * stack_ptr asm("sp");
	extern char end asm("end");
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > stack_ptr)
	{
//		write(1, "Heap and stack collision\n", 25);
//		abort();
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

int _close(int file)
{
	(void) file;
	return -1;
}


int _fstat(int file, struct stat *st)
{
	(void) file;
	(void) st;

	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	(void) file;

	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	(void) file;
	(void) ptr;
	(void) dir;

	return 0;
}

int _open(char *path, int flags, ...)
{
	(void) path;
	(void) flags;

	/* Pretend like we always fail */
	return -1;
}

int _wait(int *status)
{
	(void) status;

	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	(void) name;

	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	(void) buf;
	return -1;
}

int _stat(char *file, struct stat *st)
{
	(void) file;
	(void) st;

	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	(void) old;
	(void) new;

	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	(void) name;
	(void) argv;
	(void) env;

	errno = ENOMEM;
	return -1;
}
