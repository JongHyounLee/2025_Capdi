#include <errno.h>
#include <sys/types.h>   // off_t, ssize_t
#include <sys/unistd.h>  // _write, _read, _lseek 등 원형
#include <sys/stat.h>
#include "stm32h7xx_hal.h"

// printf를 어느 UART로 보낼지 하나로 통일하세요.
// extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;   // 현재 코드 기준

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

int _read(int file, char *ptr, int len)
{
    (void)file; (void)ptr; (void)len;
    // 필요시 구현
    return 0;
}

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;   // character device
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

// ★ off_t 사용!
off_t _lseek(int file, off_t ptr, int dir)
{
    (void)file; (void)ptr; (void)dir;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid; (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    while (1) { }
}

#include <sys/times.h>
clock_t _times(struct tms *buf)
{
    (void)buf;
    return (clock_t)-1; // not implemented
}
