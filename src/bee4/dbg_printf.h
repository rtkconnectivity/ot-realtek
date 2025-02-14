#ifndef __DBG_PRINTF_H__
#define __DBG_PRINTF_H__

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

//#include "rtl876x.h"
#include "rtl_nvic.h"
#include "rtl_uart.h"
#include "rtl_gdma.h"
#include "rtl_rcc.h"

void dbg_init(void);
int dbg_snprintf(char *buffer, size_t count, const char *format, ...);
int dbg_vprintf(const char *module, const char *fmt, va_list args);
int dbg_printf(const char *fmt, ...);
int dbg_sprintf(char *buffer, const char *format, ...);

#endif /* __DBG_PRINTF_H__ */
