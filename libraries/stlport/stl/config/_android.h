#ifndef __stl_config__android_h
#define __stl_config__android_h

#define _STLP_PLATFORM "Android"

#include <features.h>

/* The Android C library is mostly glibc-like */
#ifndef _STLP_USE_GLIBC
#  define _STLP_USE_GLIBC 1
#endif

/* ... and Unix-like */
#ifndef _STLP_UNIX
#  define _STLP_UNIX 1
#endif

/* No RTTI support in runtime */
#define _STLP_NO_TYPEINFO 1
/* No exceptions in runtime */
#define _STLP_NO_RTTI 1

/* Have pthreads support */
#define _PTHREADS

/* Don't have native <cplusplus> headers */
/* #define _STLP_HAS_NO_NEW_C_HEADERS 1 */

/* Android has defined wchar, but this functionality
   not really implemented. We think that it present,
   but see comments in Android NDK before real use
   wchars.
 */
/* Don't use wchar.h etc. */
/* #define _STLP_NO_WCHAR_T 1 */

/* Don't have (working) native wide character support */
#define _STLP_NO_NATIVE_WIDE_FUNCTIONS 1

/* Words about wchar (above) is applicable for mbstate too */
/* Don't use mbstate_t, define our own */
/* #define _STLP_NO_NATIVE_MBSTATE_T 1 */

/* No (proper) wide stream support in Android */
#define _STLP_NO_NATIVE_WIDE_STREAMS 1

/* C library is in the global namespace */
#define _STLP_VENDOR_GLOBAL_CSTD 1

/* Don't have underlying locale support. */
#undef _STLP_REAL_LOCALE_IMPLEMENTED

/* No pthread_spinlock_t in Android */
#define _STLP_DONT_USE_PTHREAD_SPINLOCK 1

/* Little endian platform. */
#define _STLP_LITTLE_ENDIAN 1

/* No <exception> headers */
#define _STLP_NO_EXCEPTION_HEADER 1

/* No need to define our own namespace */
#define _STLP_NO_OWN_NAMESPACE 1

/* Need this to define STLport's own bad_alloc class (which won't be
 * thrown in any case)
 */
#define _STLP_NEW_DONT_THROW_BAD_ALLOC 1

/* Use __new_alloc instead of __node_alloc, so we don't need static functions.
#define _STLP_USE_SIMPLE_NODE_ALLOC 1
*/

/* Don't use extern versions of range errors, so we don't need to
 * compile as a library.
#define _STLP_USE_NO_EXTERN_RANGE_ERRORS 1
*/

/* The system math library doesn't have long double variants, e.g
 * sinl, cosl, etc
 */
#define _STLP_NO_VENDOR_MATH_L 1

#define _STLP_USE_UNIX_IO 1

/* bypass arch-arm/usr/include/new */
#ifndef __NEW__
#  define __NEW__
#endif

#endif /* __stl_config__android_h */
