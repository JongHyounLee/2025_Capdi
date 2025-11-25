#include <sys/stat.h>
#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart3;

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

int _read(int file, char *ptr, int len)
{
    // 필요 시 UART 수신 구현 가능
    return 0;
}

int _close(int file)        { return -1; }
int _fstat(int file, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
int _isatty(int file)       { return 1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _getpid(void)           { return 1; }
int _kill(int pid, int sig) { return -1; }
void _exit(int status)      { while (1); }
#include <sys/times.h>

clock_t _times(struct tms *buf)
{
    return -1; // not implemented
}
