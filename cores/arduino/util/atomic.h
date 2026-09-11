/* Copyright (c) 2007 Dean Camera
   Modified for ARM Cortex, 2019 by Bill Westfield.
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
  POSSIBILITY OF SUCH DAMAGE.
*/

/*
   Modified for ARM Cortex, 2019 by Bill Westfield.
   Specifically, this is designed so that Arduino sketches and libraries
    designed using the avr-libc "util/atomic.h" facilities will be
    portable to ARM-based Arduinos.
   The internal documentation has been deleted, since it was largely
    AVR-specific.  The internal function names are still AVR-derived.
   The functions __enable_irq(), __disable_irq(), __get_PRIMASK(), and
    __set_PRIMASK() are defined by the ARM CMSIS specs, and should be
    present on all ARM Cortex processors.
*/

#ifndef _UTIL_ATOMIC_H_
#define _UTIL_ATOMIC_H_ 1

/* Internal helper functions. */
static __inline__ uint32_t __iSeiRetVal(void)
{
    __enable_irq();
    return 1;
}

static __inline__ uint32_t __iCliRetVal(void)
{
    __disable_irq();
    return 1;
}

static __inline__ void __iSeiParam(const uint32_t *__s)
{
    __enable_irq();
    __asm__ volatile ("" ::: "memory");
    (void)__s;
}

static __inline__ void __iCliParam(const uint32_t *__s)
{
    __disable_irq();
    __asm__ volatile ("" ::: "memory");
    (void)__s;
}

static __inline__ void __iRestore(const  uint32_t *__s)
{
    __set_PRIMASK(*__s);
    __asm__ volatile ("" ::: "memory");
}

#define ATOMIC_BLOCK(type) for ( type, __ToDo = __iCliRetVal(); \
	                       __ToDo ; __ToDo = 0 )

#define NONATOMIC_BLOCK(type) for ( type, __ToDo = __iSeiRetVal(); \
	                          __ToDo ;  __ToDo = 0 )

#define ATOMIC_RESTORESTATE uint32_t primask_save \
	__attribute__((__cleanup__(__iRestore))) = __get_PRIMASK()

#define ATOMIC_FORCEON uint32_t primask_save \
	__attribute__((__cleanup__(__iSeiParam))) = 0

#define NONATOMIC_RESTORESTATE uint32_t primask_save \
	__attribute__((__cleanup__(__iRestore))) = __get_PRIMASK()

#define NONATOMIC_FORCEOFF uint32_t primask_save \
	__attribute__((__cleanup__(__iCliParam))) = 0

#endif
