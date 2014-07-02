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
#ifndef __SCANNER_H__
#define __SCANNER_H__

#include "btle.h"

#include <stdbool.h>

typedef enum
{
  SCANNER_STATE_IDLE,
  SCANNER_STATE_INIT_ERROR,
  SCANNER_STATE_READY,
  SCANNER_STATE_WAIT_TO_SCAN_ADV,
  SCANNER_STATE_SCAN_ADV,
  SCANNER_STATE_WAIT_TO_SEND_REQ,
  SCANNER_STATE_SEND_REQ,
  SCANNER_STATE_WAIT_TO_SCAN_RSP,
  SCANNER_STATE_SCAN_RSP
} scanner_state_t;

btle_status_codes_t btle_scan_ev_get (btle_event_t *p_ev);
btle_status_codes_t btle_scan_param_set (btle_cmd_param_le_write_scan_parameters_t param);
btle_status_codes_t btle_scan_enable_set (btle_cmd_param_le_write_scan_enable_t param);

#endif /* __SCANNER_H__ */
