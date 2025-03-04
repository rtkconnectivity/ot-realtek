///////////////////////////////////////////////////////////////////////////////
//
// \license The MIT License (MIT)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
///////////////////////////////////////////////////////////////////////////////
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
