/*
 * Copyright (c) 1997-1999
 * Silicon Graphics Computer Systems, Inc.
 *
 * Copyright (c) 1999
 * Boris Fomitchev
 *
 * Copyright (c) 2003
 * Francois Dumont
 *
 * This material is provided "as is", with absolutely no warranty expressed
 * or implied. Any use is at your own risk.
 *
 * Permission to use or copy this software for any purpose is hereby granted
 * without fee, provided the above notices are retained on all copies.
 * Permission to modify the code and to distribute modified code is granted,
 * provided the above notices are retained, and a notice that the code was
 * modified is included with the above copyright notice.
 *
 */
#ifndef _STLP_WINCE_WINDOWS_SUFFIX_H
#define _STLP_WINCE_WINDOWS_SUFFIX_H


#if !defined (_STLP_WINDOWS_H_INCLUDED) && defined(_STLP_WCE)
#  define _STLP_WINDOWS_H_INCLUDED
#  if defined (__BUILDING_STLPORT)
#    include <stl/config/_native_headers.h>
/* Here we define _STLP_OUTERMOST_HEADER_ID to avoid indirect inclusion
 * of STLport stuffs from C/C++ Standard headers exposed by STLport
 * as configuration is not yet completed. */
#    if !defined (_STLP_OUTERMOST_HEADER_ID)
#      define _STLP_OUTERMOST_HEADER_ID 0x100
#    endif
#    if !defined (WIN32_LEAN_AND_MEAN)
#      define WIN32_LEAN_AND_MEAN
#    endif
#    if !defined (VC_EXTRALEAN)
#      define VC_EXTRALEAN
#    endif
/* Don't let windows.h define its min and max macros. */
#    if !defined (NOMINMAX)
#      define NOMINMAX
#    endif
#    if !defined (STRICT)
#      define STRICT
#    endif
#    if defined (_STLP_USE_MFC)
#      include <afx.h>
#    else
#      include <windows.h>
#    endif
#    if (_STLP_OUTERMOST_HEADER_ID == 0x100)
#      undef _STLP_OUTERMOST_HEADER_ID
#    endif
#  else
/* This section serves as a replacement for windows.h header. */
#    if defined (__cplusplus)
extern "C" {
#    endif
#    if (defined (_M_AMD64) || defined (_M_IA64) || (!defined (_STLP_WCE) && defined (_M_MRX000)) || defined (_M_ALPHA) || \
        (defined (_M_PPC) && (_STLP_MSVC_LIB >= 1000))) && !defined (RC_INVOKED)
#      define InterlockedIncrement       _InterlockedIncrement
#      define InterlockedDecrement       _InterlockedDecrement
#      define InterlockedExchange        _InterlockedExchange
#      define _STLP_STDCALL
#    else
#      if defined (_MAC)
#        define _STLP_STDCALL _cdecl
#      else
#        define _STLP_STDCALL __stdcall
#      endif
#    endif

#    if defined (_STLP_NEW_PLATFORM_SDK)
_STLP_IMPORT_DECLSPEC long _STLP_STDCALL InterlockedIncrement(long volatile *);
_STLP_IMPORT_DECLSPEC long _STLP_STDCALL InterlockedDecrement(long volatile *);
_STLP_IMPORT_DECLSPEC long _STLP_STDCALL InterlockedExchange(long volatile *, long);
#      if defined (_WIN64)
_STLP_IMPORT_DECLSPEC void* _STLP_STDCALL _InterlockedExchangePointer(void* volatile *, void*);
#      endif
#    else
/* start of eMbedded Visual C++ specific section */
#      include <stl/config/_native_headers.h>

/* Don't let windef.h define its min and max macros. */
#      if !defined (NOMINMAX)
#        define NOMINMAX
#      endif
#      include <windef.h> /* needed for basic windows types */

       /** in SDKs generated with PB5, windef.h somehow includes headers which then
       define setjmp. */
#      if (_WIN32_WCE >= 0x500)
#        define _STLP_NATIVE_SETJMP_H_INCLUDED
#      endif

#      ifndef _WINBASE_ /* winbase.h already included? */
long WINAPI InterlockedIncrement(long*);
long WINAPI InterlockedDecrement(long*);
long WINAPI InterlockedExchange(long*, long);
#      endif

#      ifndef __WINDOWS__ /* windows.h already included? */

#        if defined (x86)
#          include <winbase.h> /* needed for inline versions of Interlocked* functions */
#        endif

#        ifndef _MFC_VER

#          define MessageBox MessageBoxW
int WINAPI MessageBoxW(HWND hWnd, LPCWSTR lpText, LPCWSTR lpCaption, UINT uType);

#          define wvsprintf wvsprintfW
int WINAPI wvsprintfW(LPWSTR, LPCWSTR, va_list ArgList);

void WINAPI ExitThread(DWORD dwExitCode);

#          if !defined (COREDLL)
#            define _STLP_WCE_WINBASEAPI DECLSPEC_IMPORT
#          else
#            define _STLP_WCE_WINBASEAPI
#          endif

_STLP_WCE_WINBASEAPI int WINAPI
MultiByteToWideChar(UINT CodePage, DWORD dwFlags, LPCSTR lpMultiByteStr,
                    int cbMultiByte, LPWSTR lpWideCharStr, int cchWideChar);

_STLP_WCE_WINBASEAPI UINT WINAPI GetACP();

_STLP_WCE_WINBASEAPI BOOL WINAPI TerminateProcess(HANDLE hProcess, DWORD uExitCode);

#          define OutputDebugString OutputDebugStringW
void WINAPI OutputDebugStringW(LPCWSTR);

_STLP_WCE_WINBASEAPI void WINAPI Sleep(DWORD);

#          undef _STLP_WCE_WINBASEAPI

#        endif /* !_MFC_VER */

#      endif /* !__WINDOWS__ */

/* end of eMbedded Visual C++ specific section */
#    endif

#    if defined (InterlockedIncrement)
#      pragma intrinsic(_InterlockedIncrement)
#      pragma intrinsic(_InterlockedDecrement)
#      pragma intrinsic(_InterlockedExchange)
#      if defined (_WIN64)
#        pragma intrinsic(_InterlockedExchangePointer)
#      endif
#    endif
#    if defined (__cplusplus)
} /* extern "C" */
#    endif

#  endif

/* Here we use a macro different than the InterlockedExchangePointer SDK one
 * to avoid macro definition conflict. */
#  if !defined (_WIN64)
/* Under 32 bits platform we rely on a simple InterlockedExchange call. */
#    if defined (__cplusplus)
/* We do not define this function if we are not in a C++ translation unit just
 * because of the 'inline' keyword portability issue it would introduce. We will
 * have to fix it the day we need this function for a C translation unit.
 */
inline
void* _STLP_CALL STLPInterlockedExchangePointer(void* volatile* __a, void* __b) {
#      if defined (_STLP_MSVC)
/* Here MSVC produces warning if 64 bits portability issue is activated.
 * MSVC do not see that _STLP_ATOMIC_EXCHANGE_PTR is a macro which content
 * is based on the platform, Win32 or Win64
 */
#        pragma warning (push)
#        pragma warning (disable : 4311) // pointer truncation from void* to long
#        pragma warning (disable : 4312) // conversion from long to void* of greater size
#      endif
#      if !defined (_STLP_NO_NEW_STYLE_CASTS)
  return reinterpret_cast<void*>(InterlockedExchange(reinterpret_cast<long*>(const_cast<void**>(__a)),
                                                     reinterpret_cast<long>(__b)));
#      else
  return (void*)InterlockedExchange((long*)__a, (long)__b);
#      endif
#      if defined (_STLP_MSVC)
#        pragma warning (pop)
#      endif
}
#    endif
#  else
#    define STLPInterlockedExchangePointer _InterlockedExchangePointer
#  endif

#endif /* _STLP_WINDOWS_H_INCLUDED && _STLP_WCE */

#endif /* _STLP_WINCE_WINDOWS_SUFFIX_H */
