#ifndef NEWLIB_STUBS_H
#define NEWLIB_STUBS_H

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 *
char *__env[1] = { 0 };
char **environ = __env;
*/

int _close(int file);
void _exit(int status);
int _fstat(int file, struct stat *st);
int _isatty(int file);
//int _link(char *old, char *new);
int _lseek(int file, int ptr, int dir);
int _read(int file, char *ptr, int len);
caddr_t _sbrk(int incr);
int _stat(const char *filepath, struct stat *st);
clock_t _times(struct tms *buf);
int _unlink(char *name);
int _wait(int *status);
int _write(int file, char *ptr, int len);

#endif  // NEWLIB_STUBS_H