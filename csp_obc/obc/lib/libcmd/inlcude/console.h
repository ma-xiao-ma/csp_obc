////////////////////////////////////////////////////////////////////////////////
//	功能： 调试终端功能模块头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////

#ifndef LIB_LIBCMD_INLCUDE_CONSOLE_H_
#define LIB_LIBCMD_INLCUDE_CONSOLE_H_

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>

#include <command.h>

/* Number of history elements */
#ifndef CONSOLE_HISTORY_ELEMENTS
#define CONSOLE_HISTORY_ELEMENTS    10
#endif

/* Size of console line buffer */
#ifndef CONSOLE_BUFSIZ
#define CONSOLE_BUFSIZ              100
#endif

/**
 * Initialize the debugger console thread
 */
int console_init(void);

/**
 * Exit and restore terminal settings
 */
int console_exit(void);

/**
 * Set console hostname
 */
void console_set_hostname(char *host);

/**
 * Clear the console screen
 */
void console_clear(void);

/* Update console */
void console_update(void);

#ifndef __linux__
void debug_console(void *pvParameters);
#else
void *debug_console(void *parameters);
#endif

#endif

