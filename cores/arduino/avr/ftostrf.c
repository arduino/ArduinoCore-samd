/* ftostrf - Same as dtostrf function from avr-libc, which used single-precision

   Copyright (c) 2005, Dmitry Xmelkov
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

/* $Id: dtostrf.c 1944 2009-04-01 23:12:20Z arcanum $ */

#include <stdlib.h>
#include "ftoa_engine.h"
#include "dtoa_conv.h"

/* ???  Is 'width' a signed value?
   dtostrf.S comment:
        If precision is < 0, the string is left adjusted with leading spaces.
        If precision is > 0, the string is right adjusted with trailing spaces.
   dtostrf.S code:
        'p_width' is a register for left/right adjustment
   avr-libc manual:
        nothing about this

   So, for compatibilty 'width' is signed value to left/right adjust.
 */

char *ftostrf (float val, signed char width, unsigned char prec, char *sout)
{
    unsigned char flags;

    /* DTOA_UPPER: for compatibility with avr-libc <= 1.4 with NaNs     */
    flags = width < 0 ? DTOA_LEFT | DTOA_UPPER : DTOA_UPPER;
    dtoa_prf (val, sout, abs(width), prec, flags);
    return sout;
}
