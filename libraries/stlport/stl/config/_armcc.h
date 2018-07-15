/* STLport configuration file
 * It is internal STLport header - DO NOT include it directly
 */
#ifndef _STLP_ARMCC_H
#define  _STLP_ARMCC_H

#define _STLP_COMPILER "armcc"
#define _STLP_NO_VENDOR_STDLIB_L 1
#define _STLP_NO_VENDOR_MATH_F 1
#define _STLP_NO_VENDOR_MATH_L 1
#define _STLP_NO_CSTD_FUNCTION_IMPORTS 1
#define _STLP_HAS_NO_NEW_C_HEADERS 1
#define _STLP_USE_STDIO_IO 1
#define _STLP_NO_STAT_H 1
#define _STLP_CPP_MBSTATE_T 1
//#define _STLP_CSTDIO 1
#define _STLP_NO_OWN_NAMESPACE 1
#define _STLP_USE_NAMESPACES
#define _STLP_NO_THREADS 1
#define _STLP_USE_EXCEPTIONS 1
#define _STLP_NO_TYPEINFO 1
#define _STLP_NO_TIME_SUPPORT 1
#define _STLP_DONT_USE_PRIV_NAMESPACE 1
#define _STLP_NO_EXPLICIT_FUNCTION_TMPL_ARGS 1
#define __USE_C99_MATH
#undef _STLP_LONG_LONG
//#undef _STLP_MEMBER_TEMPLATES 
//#define _STLP_DONT_SUPPORT_REBIND_MEMBER_TEMPLATE 1
#undef _STLP_NO_UNCAUGHT_EXCEPT_SUPPORT
#undef _STLP_NO_UNEXPECTED_EXCEPT_SUPPORT
#undef _STLP_THREADS
//#define _STLP_NO_EXTENSIONS
# undef linux
# undef __linux__

#if 0
#  pragma diag_suppress 1300
#  pragma diag_suppress 111
#  pragma diag_suppress 1081
#  pragma diag_suppress 1299
#  pragma diag_suppress 186
#  pragma diag_suppress 68
#endif
#define _STLP_HAS_INCLUDE_NEXT 1

//==========================================================
#endif
