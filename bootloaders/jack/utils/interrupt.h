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
#ifndef UTILS_INTERRUPT_H
#define UTILS_INTERRUPT_H

//#include <parts.h>

#include "interrupt/interrupt_sam_nvic.h"

/**
 * \defgroup interrupt_group Global interrupt management
 *
 * This is a driver for global enabling and disabling of interrupts.
 *
 * @{
 */

#if defined(__DOXYGEN__)
/**
 * \def CONFIG_INTERRUPT_FORCE_INTC
 * \brief Force usage of the ASF INTC driver
 *
 * Predefine this symbol when preprocessing to force the use of the ASF INTC driver.
 * This is useful to ensure compatibility across compilers and shall be used only when required
 * by the application needs.
 */
#  define CONFIG_INTERRUPT_FORCE_INTC
#endif

//! \name Global interrupt flags
//@{
/**
 * \typedef irqflags_t
 * \brief Type used for holding state of interrupt flag
 */

/**
 * \def cpu_irq_enable
 * \brief Enable interrupts globally
 */

/**
 * \def cpu_irq_disable
 * \brief Disable interrupts globally
 */

/**
 * \fn irqflags_t cpu_irq_save(void)
 * \brief Get and clear the global interrupt flags
 *
 * Use in conjunction with \ref cpu_irq_restore.
 *
 * \return Current state of interrupt flags.
 *
 * \note This function leaves interrupts disabled.
 */

/**
 * \fn void cpu_irq_restore(irqflags_t flags)
 * \brief Restore global interrupt flags
 *
 * Use in conjunction with \ref cpu_irq_save.
 *
 * \param flags State to set interrupt flag to.
 */

/**
 * \fn bool cpu_irq_is_enabled_flags(irqflags_t flags)
 * \brief Check if interrupts are globally enabled in supplied flags
 *
 * \param flags Currents state of interrupt flags.
 *
 * \return True if interrupts are enabled.
 */

/**
 * \def cpu_irq_is_enabled
 * \brief Check if interrupts are globally enabled
 *
 * \return True if interrupts are enabled.
 */
//@}

//! @}

/**
 * \ingroup interrupt_group
 * \defgroup interrupt_deprecated_group Deprecated interrupt definitions
 */

#endif /* UTILS_INTERRUPT_H */
