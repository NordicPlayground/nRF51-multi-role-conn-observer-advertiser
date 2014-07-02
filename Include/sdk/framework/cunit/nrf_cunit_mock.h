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

#ifndef NRF_CUNIT_MOCK_H_
#define NRF_CUNIT_MOCK_H_

#include <stdint.h>

/**@brief Macro that verifies that the received function call was expected.
 *        To be used used by mock implementations.
 */
#define NRF_CUNIT_FIND_MOCK_CALL(pp_mock_call) \
do {\
  nrf_cunit_find_mock_call_handler((const uint8_t *)__FILE__, (const uint8_t *)__func__, \
                                   (uint16_t)__LINE__, pp_mock_call); \
} while (0);


/**@brief Mock call compare rules can be used to tell a given mock obhject extended rules for
 * comparing, such as STRICT compare, ANY compare, NO compare.
 */
typedef enum
{
    COMPARE_STRICT,             /**< Value must have exactly match, like 5 = 5. (Default rule). */
    COMPARE_RANGE_10,           /**< Valid value must be provided and be within +/- 10 from expected value. */
    COMPARE_ANY,                /**< Valid value must be provided (e.g. test against NULL), but any value is accepted, can be used to ensure that a given function was called with some parameters, but without verifying exact value. */
    COMPARE_NOT_NULL,           /**< Validate that an argument is not a NULL pointer. */
    COMPARE_POINTERS,           /**< Validate that the pointer is expected, and not the content pointed to by the pointer. */
    COMPARE_NO                  /**< No validation done for parameters. */
} nrf_cunit_mock_compare_rule_t;

/**@brief Mock call structure used for registering expected calls and returning results.
 *        It also contains an error field, that can be used for requesting the mock to return
 *        an error for error handling verification.
 */
typedef struct
{
    uint8_t  cmd[32];           /**< Name of function that is expected to be executed. */
    uint32_t arg[4];            /**< Argument(s) that should be expected to be received when unit calls this mock. */
    uint32_t arg_actual[4];     /**< Argument(s) that was used when unit called this mock. */
    uint32_t result[4];         /**< Result(s) to be returned to unit calling the mock. */
    uint32_t error;             /**< Error to be returned to calling unit when the mock is executed, used for error handling verification process. */
    uint32_t executed;          /**< false, when function has not been called by unit being tested, true when this mock has executed. */
    nrf_cunit_mock_compare_rule_t compare_rule[4]; /**< Compare rule for each argument 0-3 for given function, default is: COMPARE_STRICT. */
} nrf_cunit_mock_call;


/**
 * @brief nrf_cunit_mock_file_as_stub is a generic function call to handle mock implementation.
 *
 * @details It will register the file name provided in its internal database and whenever a 
 *          (mock)-function in p_file_name is called, then it will be treated as if the call 
 *          was made to a stub. Thus no validation on arguments are performed and NRF_SUCCESS 
 *          is always returned for any calls to the mock in question.
 *
 *          @note When \ref nrf_cunit_reset_mock is called, all files currently treated as stubs 
                  will be nulled, and thus used as mocks.
 *
 * @param[in]   p_file_name     Name of mock file that should be treated as a simple stub.
 */
void nrf_cunit_mock_file_as_stub(uint8_t * p_file_name);


/**
 * @brief The nrf_cunit_expect_call_return is a generic function call to handle mock implementation.
 *
 * @details It will register the expected call in its internal database, return a pointer to the
 *          nrf_cunit_mock_call where the unit test can setup the mock behaviour.
 *
 * @param[in]   p_function      Name of function for wich a mock call should be expected
 * @param[out]  pp_mock_call    A pointer will be returned to the unit test with the structure that
 *                              will be used when mock is called.
 */
void nrf_cunit_expect_call_return(uint8_t * p_function, nrf_cunit_mock_call **pp_mock_call);


/**
 * @brief   The nrf_cunit_verify_call_return is a generic function call to verify the behaviour of
 *          the unit being tested.
 *
 * It will loop the mock call database and verify that all mock structeres registered has been
 * called by the unit being tested, as expected by the unit test.
 * All unexecuted functions will generate a unit test failure.
 */
void nrf_cunit_verify_call_return(void);


/**
 * @brief nrf_cunit_reset_mock will clear the mock database.
 * Used when preparing a new unit test.
 */
void nrf_cunit_reset_mock(void);


/**
 * @brief Used internally by nrf_cunit_mock.c. Mock implementations should use the
 *        NRF_CUNIT_FIND_MOCK_CALL macro.
 */
void nrf_cunit_find_mock_call_handler(const uint8_t * p_file,
                                      const uint8_t * p_function,
                                      uint16_t line_num,
                                      nrf_cunit_mock_call **pp_mock_call);


/**@brief Function that stores the arguments received in the mock call.
 *        To be used AFTER the arguments of the mock call have been verified.
 */
void nrf_cunit_save_mock_call_arg(nrf_cunit_mock_call * p_mock_call,
                                  uint32_t arg0,
                                  uint32_t arg1,
                                  uint32_t arg2,
                                  uint32_t arg3);

#endif /* NRF_CUNIT_MOCK_H_ */
