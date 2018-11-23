#ifndef __SMARTCARD_PRINT_H__
#define __SMARTCARD_PRINT_H__

#include "autoconf.h"
#include "api/print.h"

#define SMARTCARD_DEBUG CONFIG_SMARTCARD_DEBUG
#define MEASURE_TOKEN_PERF

/* Primitive for debug output */
#if SMARTCARD_DEBUG
#define log_printf(...) printf(__VA_ARGS__)
#else
#define log_printf(...)
#endif

#endif /* _SMARTCARD_PRINT_H__ */
