/**
 * \file
 *
 * \brief SAM RTC Count Quick Start
 *
 * Copyright (C) 2013-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \page asfdoc_sam0_rtc_count_callback_use_case Quick Start Guide for RTC (COUNT) - Callback
 *
 * In this use case, the RTC is set up in count mode. The quick start
 * configures the RTC in 16 bit mode and to continuously update COUNT register.
 * The rest of the configuration is according to the
 * \ref rtc_count_get_config_defaults "default".
 * A callback is implemented for when the RTC overflows.
 *
 * \section asfdoc_sam0_rtc_count_callback_use_case_prereq Prerequisites
 * The Generic Clock Generator for the RTC should be configured and enabled; if
 * you are using the System Clock driver, this may be done via \c conf_clocks.h.
 *
 * \subsection asfdoc_sam0_rtc_count_callback_use_case_setup_clocks Clocks and Oscillators
 * The \c conf_clock.h file needs to be changed with the following values to
 * configure the clocks and oscillators for the module.
 *
 * The following oscillator settings are needed:
 * \snippet conf_clocks.h oscillator_settings
 * The following generic clock settings are needed:
 * \snippet conf_clocks.h gclk_settings
 *
 * \section asfdoc_sam0_rtc_count_callback_use_case_setup Setup
 *
 * \subsection asfdoc_sam0_rtc_count_callback_use_case_setup_code Code
 * Create a rtc_module struct and add to the main application source file,
 * outside of any functions:
 * \snippet qs_rtc_count_callback.c rtc_module_instance
 * The following must be added to the user application:
 *
 * Function for setting up the module:
 * \snippet qs_rtc_count_callback.c initialize_rtc
 *
 * Callback function:
 * \snippet qs_rtc_count_callback.c callback
 *
 * Function for setting up the callback functionality of the driver:
 * \snippet qs_rtc_count_callback.c setup_callback
*
 * Add to user application main():
 * \snippet qs_rtc_count_callback.c run_initialize_rtc
 *
 * \subsection asfdoc_sam0_rtc_count_callback_use_case_setup_workflow Workflow
 * -# Initialize system.
 *    \snippet qs_rtc_count_callback.c system_init
 * -# Configure and enable module.
 *    \snippet qs_rtc_count_callback.c run_conf
 * -# Create a RTC configuration structure to hold the desired RTC driver
 *    settings and fill it with the default driver configuration values.
 *    \snippet qs_rtc_count_callback.c init_conf
 *    \note This should always be performed before using the configuration
 *          struct to ensure that all values are initialized to known default
 *          settings.
 *
 * -# Alter the RTC driver configuration to run in 16-bit counting mode, with
 *    continuous counter register updates and a compare value of 1000ms.
 *    \snippet qs_rtc_count_callback.c set_config
 * -# Initialize the RTC module.
 *    \snippet qs_rtc_count_callback.c init_rtc
 * -# Enable the RTC module, so that it may begin counting.
 *    \snippet qs_rtc_count_callback.c enable
 * -# Configure callback functionality.
 *    \snippet qs_rtc_count_callback.c run_callback
 *  -# Register overflow callback.
 *     \snippet qs_rtc_count_callback.c reg_callback
 *  -# Enable overflow callback.
 *     \snippet qs_rtc_count_callback.c en_callback
 * -# Set period.
 *    \snippet qs_rtc_count_callback.c period
 *
 * \section asfdoc_sam0_rtc_count_callback_use_case_implementation Implementation
 *
 * \subsection asfdoc_sam0_rtc_count_callback_use_case_implementation_code Code
 * Add to user application main:
 * \snippet qs_rtc_count_callback.c while
 * \subsection asfdoc_sam0_rtc_count_callback_use_case_implementation_workflow Workflow
 * -# Infinite while loop while waiting for callbacks.
 *    \snippet qs_rtc_count_callback.c while
 *
 * \section asfdoc_sam0_rtc_count_callback_use_case_callback Callback
 * Each time the RTC counter overflows, the callback function will be called.
 * \subsection asfdoc_sam0_rtc_count_callback_use_case_callback_workflow Workflow
 * -# Perform the desired user action for each RTC overflow:
 *    \snippet qs_rtc_count_callback.c overflow_act
 */
