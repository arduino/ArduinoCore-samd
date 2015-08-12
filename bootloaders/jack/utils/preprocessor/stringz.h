/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011-2012, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifndef _STRINGZ_H_
#define _STRINGZ_H_

/**
 * \defgroup group_sam0_utils_stringz Preprocessor - Stringize
 *
 * \ingroup group_sam0_utils
 *
 * @{
 */

/** \brief Stringize.
 *
 * Stringize a preprocessing token, this token being allowed to be \#defined.
 *
 * May be used only within macros with the token passed as an argument if the
 * token is \#defined.
 *
 * For example, writing STRINGZ(PIN) within a macro \#defined by PIN_NAME(PIN)
 * and invoked as PIN_NAME(PIN0) with PIN0 \#defined as A0 is equivalent to
 * writing "A0".
 */
#define STRINGZ(x)                                #x

/** \brief Absolute stringize.
 *
 * Stringize a preprocessing token, this token being allowed to be \#defined.
 *
 * No restriction of use if the token is \#defined.
 *
 * For example, writing ASTRINGZ(PIN0) anywhere with PIN0 \#defined as A0 is
 * equivalent to writing "A0".
 */
#define ASTRINGZ(x)                               STRINGZ(x)

/** @} */

#endif  // _STRINGZ_H_
