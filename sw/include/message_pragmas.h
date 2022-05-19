#ifndef MESSAGE_PRAGMAS_H
#define MESSAGE_PRAGMAS_H

/* some helper macros */
#define DO_PRAGMA(x) _Pragma (#x)
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)

/* some convenience macros to print debug/config messages at compile time */
#if defined(PPRZ_TESTS)
#define WARNING(x)
#define MESSAGE(x)
#define TODO(x)
#define INFO(x)
#define INFO_VALUE(x,v)
#define INFO_VAR(var)
#else
#define WARNING(x) DO_PRAGMA(GCC warning #x)
#define MESSAGE(x) DO_PRAGMA(message (x))
#define TODO(x) DO_PRAGMA(message ("TODO - " x))
#define INFO(x) DO_PRAGMA(message ("Info: " x))
#define INFO_VALUE(x,v) DO_PRAGMA(message ("Info: " x VALUE(v)))
#define INFO_VAR(var) DO_PRAGMA(message ("INFO: " #var " = " VALUE(var)))
#endif

/* only if PRINT_CONFIG is true */
#if PRINT_CONFIG
#define PRINT_CONFIG_MSG(x) DO_PRAGMA(message ("Config: " x))
#define PRINT_CONFIG_MSG_VALUE(x,v) DO_PRAGMA(message ("Config: " x VALUE(v)))
#define PRINT_CONFIG_VAR(var) DO_PRAGMA(message ("Config: " #var " = " VALUE(var)))
#else
#define PRINT_CONFIG_MSG(x)
#define PRINT_CONFIG_MSG_VALUE(x,v)
#define PRINT_CONFIG_VAR(var)
#endif

#endif
