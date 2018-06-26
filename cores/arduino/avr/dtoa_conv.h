/* Copyright (c) 2005, Dmitry Xmelkov
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

/* $Id: dtoa_conv.h 1175 2007-01-14 15:18:18Z dmix $ */

#ifndef	_DTOA_CONV_H
#define	_DTOA_CONV_H

#include <stdio.h>

int dtoa_prf (float val, char *s, unsigned char width, unsigned char prec,
              unsigned char flags);

#define	DTOA_SPACE  0x01	/* put space for positives	*/
#define	DTOA_PLUS   0x02	/* put '+' for positives	*/
#define	DTOA_UPPER  0x04	/* use uppercase letters	*/
#define	DTOA_ZFILL  0x08	/* fill zeroes			*/
#define	DTOA_LEFT   0x10	/* adjust to left		*/
#define	DTOA_NOFILL 0x20	/* do not fill to width		*/
#define	DTOA_EXP    0x40	/* d2stream: 'e(E)' format	*/
#define	DTOA_FIX    0x80	/* d2stream: 'f(F)' format	*/

#define	DTOA_EWIDTH     (-1)	/* Width too small	*/
#define	DTOA_NONFINITE  (-2)	/* Value is NaN or Inf	*/

#endif	/* !_DTOA_CONV_H */
