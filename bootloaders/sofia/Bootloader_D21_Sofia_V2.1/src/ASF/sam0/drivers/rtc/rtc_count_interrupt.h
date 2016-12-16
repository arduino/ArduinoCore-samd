/**
 * \file
 *
 * \brief SAM RTC Driver (Count Interrupt Mode)
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

#ifndef RTC_COUNT_INTERRUPT_H_INCLUDED
#define RTC_COUNT_INTERRUPT_H_INCLUDED

#include "rtc_count.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup asfdoc_sam0_rtc_count_group
 * @{
 */

 /**
 * \name Callbacks
 * @{
 */
enum status_code rtc_count_register_callback(
		struct rtc_module *const module,
		rtc_count_callback_t callback,
		enum rtc_count_callback callback_type);

enum status_code rtc_count_unregister_callback(
		struct rtc_module *const module,
		enum rtc_count_callback callback_type);

void rtc_count_enable_callback(
		struct rtc_module *const module,
		enum rtc_count_callback callback_type);

void rtc_count_disable_callback(
		struct rtc_module *const module,
		enum rtc_count_callback callback_type);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* RTC_COUNT_INTERRUPT_H_INCLUDED */
