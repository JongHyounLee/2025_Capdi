#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include "stm32h7xx_hal.h"
#include "usart.h"

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

int _read(int file, char *ptr, int len)
{
    HAL_UART_Receive(&huart1, (uint8_t *)ptr, 1, HAL_MAX_DELAY);
    return 1;
}

int _close(int file)
{
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}
