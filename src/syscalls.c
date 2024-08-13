/**
 ******************************************************************************
 * @file      syscalls.c
 * @author    Auto-generated by STM32CubeIDE
 * @brief     STM32CubeIDE Minimal System calls file
 *
 *            For more information about which c-functions
 *            need which of these lowlevel functions
 *            please consult the Newlib libc-manual
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2020-2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>


#define UNLOCK_ITM_REG			0xC5ACCE55

//debug registers
#define DEBUG_REG_DEMCR			((volatile uint32_t*)0xE000EDFC) //Debug Exception and Monitor Control Register

//DEMCR bits
#define TRCENA 					24

#define ITM_LOCK_ACCESS_REG		((volatile uint32_t*)0xE0000FB0)

//ITM registers

#define DEBUG_REG_ITM_TER		((volatile uint32_t*)0xE0000E00)
#define DEBUG_REG_ITM_TPR		((volatile uint32_t*)0xE0000E40)
#define DEBUG_REG_ITM_TCR		((volatile uint32_t*)0xE0000E80)

typedef struct{
volatile uint32_t ITM_STIM[32];		//Stimulus Port Registers 0-31
}ITM_St_reg_t;

#define DEBUG_REG_ITM_STIM		((ITM_St_reg_t*)0xE0000000)


/* Variables */
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

// ITM function
//start logging by clicking on the red button in SWV ITM data console
void ITM_SendChar(uint8_t log){

	//The bit TRCEN of the Debug Exception and Monitor Control register must be enabled
	//before programming or using the ITM.
	(*DEBUG_REG_DEMCR) |= (1<<TRCENA);

	//enable logging in DEBUG_CR register
	(*((volatile uint32_t*)0xE0042004)) |= 1<<5;

	//unlock ITM write access
	(*ITM_LOCK_ACCESS_REG) = UNLOCK_ITM_REG;

	//enable stimulus port 0
	(*DEBUG_REG_ITM_TER) |= (1<<0);

	//enable ITM Trace Privilege register to unmask stimulus ports 7:0
	(*DEBUG_REG_ITM_TPR) |= (1<<0);

	// read FIFO status in bit [0]:
	while(!(DEBUG_REG_ITM_STIM->ITM_STIM[0] & 1));

	//write the character to the port
	DEBUG_REG_ITM_STIM->ITM_STIM[0] = log;
}


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
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}

void _exit (int status)
{
  _kill(status, -1);
  while (1) {}    /* Make sure we hang here */
}

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    *ptr++ = __io_getchar();
  }

  return len;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    //__io_putchar(*ptr++);
	  ITM_SendChar(*ptr++);
  }
  return len;
}

int _close(int file)
{
  (void)file;
  return -1;
}


int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

int _open(char *path, int flags, ...)
{
  (void)path;
  (void)flags;
  /* Pretend like we always fail */
  return -1;
}

int _wait(int *status)
{
  (void)status;
  errno = ECHILD;
  return -1;
}

int _unlink(char *name)
{
  (void)name;
  errno = ENOENT;
  return -1;
}

int _times(struct tms *buf)
{
  (void)buf;
  return -1;
}

int _stat(char *file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _link(char *old, char *new)
{
  (void)old;
  (void)new;
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
  (void)name;
  (void)argv;
  (void)env;
  errno = ENOMEM;
  return -1;
}
