/* Copyright (c) 2014, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   * Neither the name of Nordic Semiconductor ASA nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __LL_SCANNER_H__
#define __LL_SCANNER_H__

#include "btle.h"

#include <stdbool.h>

/** @brief Callback for events on the radio */
void ll_scan_radio_cb (bool crc_valid);
void ll_scan_timer_cb (void);

btle_status_codes_t ll_scan_init (void);
btle_status_codes_t ll_scan_reset (void);
btle_status_codes_t ll_scan_start (void);
btle_status_codes_t ll_scan_stop (void);

btle_status_codes_t ll_scan_prepare (btle_scan_types_t scan_type, btle_address_type_t address_type, btle_scan_filter_policy_t filter_policy);
btle_status_codes_t ll_scan_type_configure (btle_scan_types_t scan_type);
btle_status_codes_t ll_scan_address_type_configure (btle_address_type_t own_address_type);
btle_status_codes_t ll_scan_filter_policy_configure (btle_scan_filter_policy_t scanning_filter_policy);

#endif /* __LL_SCANNER_H__ */
