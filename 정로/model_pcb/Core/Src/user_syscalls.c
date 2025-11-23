#include "stm32h7xx_hal.h"
#include "usart.h"
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/unistd.h>
extern UART_HandleTypeDef huart5;  // printf UART 사용 포트 (필요에 맞게 수정)

// write() 함수 재정의 → printf()를 UART로 리다이렉트
int _write(int file, char *ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        HAL_UART_Transmit(&huart5, (uint8_t*)ptr, len, HAL_MAX_DELAY);
        return len;
    }
    return -1;
}

// 아래 함수들은 사용하지 않더라도 링커 경고 방지를 위해 기본 구현 추가
int _read(int file, char *ptr, int len) { return 0; }
int _close(int file) { return -1; }
int _fstat(int file, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { return 1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _kill(int pid, int sig) { return -1; }
int _getpid(void) { return 1; }
