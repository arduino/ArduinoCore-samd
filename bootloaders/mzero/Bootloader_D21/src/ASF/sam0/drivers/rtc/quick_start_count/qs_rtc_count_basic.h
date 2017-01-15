/**
 * \file
 *
 * \brief SAM RTC Count Quick Start
 *
 * Copyright (C) 2012-2014 Atmel Corporation. All rights reserved.
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
 * \page asfdoc_sam0_rtc_count_basic_use_case Quick Start Guide for RTC (COUNT) - Basic
 * In this use case, the RTC is set up in count mode. The example configures the
 * RTC in 16 bit mode, with continuous updates to the COUNT register, together
 * with a set compare register value. Every 2000ms a LED on the board is
 * toggled.
 *
 * \section asfdoc_sam0_rtc_count_basic_use_case_prereq Prerequisites
 * The Generic Clock Generator for the RTC should be configured and enabled; if
 * you are using the System Clock driver, this may be done via \c conf_clocks.h.
 *
 * \subsection asfdoc_sam0_rtc_count_basic_use_case_setup_clocks Clocks and Oscillators
 * The \c conf_clock.h file needs to be changed with the following values to
 * configure the clocks and oscillators for the module.
 *
 * The following oscillator settings are needed:
 * \snippet conf_clocks.h oscillator_settings
 * The following generic clock settings are needed:
 * \snippet conf_clocks.h gclk_settings
 *
 * \section asfdoc_sam0_rtc_count_basic_use_case_setup Setup
 *
 * \subsection asfdoc_sam0_rtc_count_basic_use_case_init_code Initialization Code
 * Create a rtc_module struct and add to the main application source file,
 * outside of any functions:
 * \snippet qs_rtc_count_basic.c rtc_module_instance
 *
 * Copy-paste the following setup code to your applications \c main():
 * \snippet qs_rtc_count_basic.c initiate
 *
 * \subsection asfdoc_sam0_rtc_count_basic_use_case_main_code Add to Main
 * Add the following to your \c main().
 * \snippet qs_rtc_count_basic.c add_main
 *
 * \subsection rtc_count_basic_use_workflow Workflow
 * -# Create a RTC configuration structure to hold the desired RTC driver
 *    settings.
 *    \snippet qs_rtc_count_basic.c set_conf
 * -# Fill the configuration structure with the default driver configuration.
 *    \snippet qs_rtc_count_basic.c get_default
 *    \note This should always be performed before using the configuration
 *          struct to ensure that all values are initialized to known default
 *          settings.
 *
 * -# Alter the RTC driver configuration to run in 16-bit counting mode, with
 *    continuous counter register updates.
 *    \snippet qs_rtc_count_basic.c set_config
 * -# Initialize the RTC module.
 *    \snippet qs_rtc_count_basic.c init_rtc
 * -# Enable the RTC module, so that it may begin counting.
 *    \snippet qs_rtc_count_basic.c enable
 *
 * \section asfdoc_sam0_rtc_count_basic_use_case_implement Implementation
 * Code used to implement the initialized module.
 *
 * \subsection asfdoc_sam0_rtc_count_basic_use_case_imp_code Code
 * Add after initialization in main().
 * \snippet qs_rtc_count_basic.c implementation_code
 *
 * \subsection asfdoc_sam0_rtc_count_basic_use_case_imp_workflow Workflow
 * -# Set RTC period to 2000ms (2 seconds) so that it will overflow and reset
 *    back to zero every two seconds.
 *    \snippet qs_rtc_count_basic.c period
 * -# Enter an infinite loop to poll the RTC driver to check when a comparison
 *    match occurs.
 *    \snippet qs_rtc_count_basic.c main_loop
 * -# Check if the RTC driver has found a match on compare channel 0 against the
 *    current RTC count value.
 *    \snippet qs_rtc_count_basic.c check_match
 * -# Once a compare match occurs, perform the desired user action.
 *    \snippet qs_rtc_count_basic.c compare_match_action
 * -# Clear the compare match, so that future matches may occur.
 *    \snippet qs_rtc_count_basic.c clear_compare_match
 */
