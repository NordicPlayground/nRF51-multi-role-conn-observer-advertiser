/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "nrf_cunit_mock.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>  
#include <string.h>
#include "nrf_cunit.h"



#define MOCK_FAILURE_NO_CALL         "FAILURE - Expected mock call never occurred: "
#define MOCK_FAILURE_UNEXPECTED_CALL "FAILURE - Unexpected mock call: "

static uint32_t                      m_mock_call_index;
static nrf_cunit_mock_call           m_mock_call_db[300];
static uint32_t                      m_stub_call_index;
static nrf_cunit_mock_call           m_stub_call_db[10];


static void nrf_cunit_unexpected_mock_call(const uint8_t * p_file,
                                           const uint8_t * p_function,
                                           uint16_t line_num)
{
    uint8_t   error[255] = {0};
    uint8_t * error_idx  = error;

    memcpy(error, MOCK_FAILURE_UNEXPECTED_CALL, strlen(MOCK_FAILURE_UNEXPECTED_CALL));
    error_idx += strlen(MOCK_FAILURE_UNEXPECTED_CALL);
    memcpy(error_idx, p_function, strlen((const char *) p_function));
    cu_assert_handler(p_file, line_num, error, CU_NONFATAL);
}


void nrf_cunit_reset_mock()
{
    m_mock_call_index = 0;
    memset(m_mock_call_db, 0, sizeof(m_mock_call_db));

    m_stub_call_index = 0;
    memset(m_stub_call_db, 0, sizeof(m_stub_call_db));
}


void nrf_cunit_expect_call_return(uint8_t * cmd, nrf_cunit_mock_call ** pp_mock_call)
{
    *pp_mock_call = &m_mock_call_db[m_mock_call_index++];
    memset(*pp_mock_call, 0, sizeof(nrf_cunit_mock_call));
    strcpy((char*)(*pp_mock_call)->cmd, (const char*) cmd);
    (*pp_mock_call)->compare_rule[0] = COMPARE_STRICT;
    (*pp_mock_call)->compare_rule[1] = COMPARE_STRICT;
    (*pp_mock_call)->compare_rule[2] = COMPARE_STRICT;
    (*pp_mock_call)->compare_rule[3] = COMPARE_STRICT;
    (*pp_mock_call)->executed        = false;
}


void nrf_cunit_mock_file_as_stub(uint8_t * p_file_name)
{
    nrf_cunit_mock_call * pp_stub_call = &m_stub_call_db[m_stub_call_index++];
    
    memset(pp_stub_call, 0, sizeof(nrf_cunit_mock_call));
    strcpy((char*)pp_stub_call->cmd, (const char*) p_file_name);
}


void nrf_cunit_find_mock_call_handler(const uint8_t * p_file,
                                      const uint8_t * p_function,
                                      uint16_t line_num,
                                      nrf_cunit_mock_call **pp_mock_call)
{
    uint32_t idx;

    for (idx = 0;idx < m_stub_call_index; idx++)
    {
        if (strstr((const char *) p_file, (const char*) m_stub_call_db[idx].cmd) != NULL)
        {
            *pp_mock_call = NULL;
            return;
        }
    }

    for (idx = 0;idx < m_mock_call_index; idx++)
    {
        if (strcmp((const char *) p_function, (const char*) m_mock_call_db[idx].cmd) == 0 &&
           m_mock_call_db[idx].executed == false)
        {
            *pp_mock_call = &m_mock_call_db[idx];

            m_mock_call_db[idx].executed = true;
            return;
        }
    }

    *pp_mock_call = NULL;
    nrf_cunit_unexpected_mock_call(p_file, p_function, line_num);
}


void nrf_cunit_verify_call_return()
{
    uint32_t    idx         = 0;
    uint8_t     error[255]  = {0};
    uint32_t    err_msg_idx = 0;

    for (; idx < m_mock_call_index; idx++)
    {
        if (m_mock_call_db[idx].executed == false)
        {
            err_msg_idx = 0;
            memcpy(error + err_msg_idx, MOCK_FAILURE_NO_CALL, strlen(MOCK_FAILURE_NO_CALL));
            err_msg_idx += strlen(MOCK_FAILURE_NO_CALL);
            memcpy((error + err_msg_idx), m_mock_call_db[idx].cmd, strlen((const char *) m_mock_call_db[idx].cmd));

            cu_assert_handler((const uint8_t *)__FILE__, (uint16_t)__LINE__, error, CU_NONFATAL);
        }
    }
}


void nrf_cunit_save_mock_call_arg(nrf_cunit_mock_call * p_mock_call, uint32_t arg0, uint32_t arg1,
                                  uint32_t arg2, uint32_t arg3)
{
    p_mock_call->arg_actual[0] = arg0;
    p_mock_call->arg_actual[1] = arg1;
    p_mock_call->arg_actual[2] = arg2;
    p_mock_call->arg_actual[3] = arg3;
}

