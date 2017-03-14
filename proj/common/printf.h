
#pragma once

#ifdef WIN32
#include <stdio.h>
#else
int my_printf(const char *fmt, ...);
int my_sprintf(char* s, const char *fmt, ...);

#define printf	my_printf
#define sprintf	my_sprintf


// for debug usage with uart
/*
#define DEBUG
#ifdef  DEBUG
#define dbg_printf(FORMAT, ...) \
    printf("%s() in %s, line %i: " FORMAT, \
      __func__, __FILE__, __LINE__, ##__VA_ARGS__)
#define dbg_puts(MSG) dbg_printf("%s", MSG)
#else
#define dbg_printf(FORMAT, ...) ((void)0)
#define dbg_puts(MSG) ((void)0)
#endif
*/

#endif

