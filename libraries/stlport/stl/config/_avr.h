#define _STLP_PLATFORM "Embedded AVR-libc"

// Platform define
#define _STLP_AVR 1

// configuration defines


//#define _STLP_DONT_USE_EXCEPTIONS 1
#define _NOTHREADS 1
#define _STLP_HAS_NO_EXCEPTIONS //  compiler does not support exceptions
#define _STLP_HAS_NO_NEW_C_HEADERS 1 // native new-style C library headers lile <cstddef>, etc are not available.
#define _STLP_NEW_DONT_THROW_BAD_ALLOC  1 // compiler do not throw bad_alloc from the new operator
#define _STLP_NO_64_BIT_UINT 1 // No 64-bit unsigned integer type: disables some float stream stuff
#define _STLP_NO_AT_MEMBER_FUNCTION 1 // disable at() member functions for containers
#define _STLP_NO_BAD_ALLOC
#define _STLP_NO_DEFAULT_STREAMS 1 // disable creation of cout/cerr/cin
#define _STLP_NO_EXCEPTION_HEADER 1 // compiler lacks <exception> header
#define _STLP_NO_FSTREAM 1 // disable file streams
#define _STLP_NO_LOCALE_SUPPORT
#define _STLP_NO_LONG_DOUBLE 1 // architecture has no long double support
#define _STLP_NO_OWN_NAMESPACE
#define _STLP_NO_NATIVE_MBSTATE_T 1 // No mbstate_t defined by base library
#define _STLP_NO_NATIVE_WIDE_FUNCTIONS 1
#define _STLP_NO_NATIVE_WIDE_STREAMS 1
#define _STLP_NO_NEW_NEW_HEADER 1 // base library has no <new> header
#define _STLP_NO_RTTI 1 // compiler has no rtti support or if it has been disabled
#define _STLP_NO_THREADS 1
#define _STLP_NO_TIME_SUPPORT 1
#define _STLP_NO_TYPEINFO 1 // there is no native type_info definition
#define _STLP_NO_VENDOR_MATH_L // No long definitions in math.h
#define _STLP_NO_VENDOR_STDLIB_L // No long definitions in stdlib.h
#define _STLP_NO_WCHAR_T  1 // No wchar_t type defined
#define _STLP_USE_STATIC_LIB 1
#define _STLP_USE_STDIO_IO 1

#define _STLP_WEAK __attribute__((weak))

//#define _STLP_HAS_NO_EXCEPTIONS 1
//#undef _STLP_THREADS
//#undef _STLP_USE_DYNAMIC_LIB


//#define   _STLP_NO_AT_MEMBER_FUNCTION 1
