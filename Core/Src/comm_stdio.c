#include "comm_stdio.h"

#include <stdio.h>
#include <rt_misc.h>

#pragma import(__use_no_semihosting_swi)

struct __FILE
{
  int handle;
};

FILE __stdout;
FILE __stdin;

static UART_HandleTypeDef *g_stdio_uart;
static uint8_t g_stdio_output_enabled;

static int CommStdio_PutChar(int ch)
{
  uint8_t byte = (uint8_t)ch;

  if ((g_stdio_uart != NULL) && (g_stdio_output_enabled != 0U))
  {
    (void)HAL_UART_Transmit(g_stdio_uart, &byte, 1U, 20U);
  }
  return ch;
}

void CommStdio_Init(UART_HandleTypeDef *huart, uint8_t output_enabled)
{
  g_stdio_uart = huart;
  g_stdio_output_enabled = (output_enabled != 0U) ? 1U : 0U;
}

int fputc(int ch, FILE *file)
{
  (void)file;
  return CommStdio_PutChar(ch);
}

int fgetc(FILE *file)
{
  (void)file;
  return EOF;
}

int ferror(FILE *file)
{
  (void)file;
  return EOF;
}

void _ttywrch(int ch)
{
  (void)CommStdio_PutChar(ch);
}

void _sys_exit(int return_code)
{
  (void)return_code;
  while (1)
  {
  }
}
