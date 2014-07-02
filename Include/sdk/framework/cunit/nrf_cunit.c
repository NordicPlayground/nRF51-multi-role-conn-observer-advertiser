/*
 * Copyright (c) 2006 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/** @file
 *
 * Source file for the cUnit framework. 
 */
#include <string.h>
#include <stdbool.h>
#include "nrf_cunit.h"

#ifdef WIN32
 #include <stdlib.h>
 #include <stdio.h>
#endif

static cu_test_setup_function_t    cu_test_setup;
static cu_test_teardown_function_t cu_test_teardown;

static void cu_setup_func_default(void)
{
  ;
}

static void cu_teardown_func_default(void)
{
  ;
}

void cu_set_suite_setup_func(cu_test_setup_function_t func)
{
    cu_test_setup = func;
}

void cu_call_setup_func(void)
{
    cu_test_setup();
}

void cu_set_suite_teardown_func(cu_test_teardown_function_t func)
{
    cu_test_teardown = func;
}

void cu_call_teardown_func(void)
{
    cu_test_teardown();
}

// If are using Keil, we would have to make the send char callback visible.
#ifndef WIN32
	extern void sendchar(int ch); //lint -e526 Symbol 'sendchar(int)' not defined
#endif


static void m_put_string(uint8_t const * string)
{
#ifdef WIN32
    printf("%s", (char const *)string);
#else
    while(*string != '\0')
    {
      sendchar(*string++);
    }
#endif
}

static void m_put_decword(uint16_t w)
{
#ifdef WIN32
    printf("%d", w);
#else
  uint8_t w0;
  uint8_t w1;
  uint8_t w2;
  uint8_t w3;

  w0 = (w % 10); //Remainder of w when divided by 10
  w /= 10;       // forces w into the range [0 6553]

  w1 = (w % 10); //Remainder of w when divided by 10
  w /= 10;       // forces w into the range [0 655]

  w2 = (w % 10); //Remainder of w when divided by 10
  w /= 10;       // forces w into the range [0 65]

  w3 = (w % 10); //Remainder of w when divided by 10
  w /= 10;       // forces w into the range [0 6]

  if(w != 0)
  {
    sendchar((uint8_t)w  + '0');   /* We may safely cast w to the smaller type, as we have */
                                           /* made sure (above) that its value will fit. */
    sendchar(w3 + '0');
    sendchar(w2 + '0');
    sendchar(w1 + '0');
    sendchar(w0 + '0');
  }
  else if (w3 != 0)
  {
    sendchar(w3 + '0');
    sendchar(w2 + '0');
    sendchar(w1 + '0');
    sendchar(w0 + '0');
  }
  else if (w2 != 0)
  {
    sendchar(w2 + '0');
    sendchar(w1 + '0');
    sendchar(w0 + '0');
  }
  else if (w1 != 0)
  {
    sendchar(w1 + '0');
    sendchar(w0 + '0');
  }
  else
  {
    sendchar(w0 + '0');
  }
#endif
}

static bool     m_cu_suite_running;
static bool     m_cu_test_running;
static bool     m_cu_reg_running;
static bool     m_cu_summary_enabled  = false;
static bool     m_cu_test_failed      = false;
static uint16_t error_num             = 0;
static uint16_t m_num_of_tests;
static uint16_t m_num_of_passed_tests;
static uint16_t m_num_of_failed_tests;

void cu_registry_construct(uint8_t const  * const registry_name, uint8_t const  * const file_name)
{
  m_cu_reg_running      = true;
  m_cu_suite_running    = false;
  m_cu_test_running     = false;
  m_num_of_tests        = 0;
  m_num_of_passed_tests = 0;
  m_num_of_failed_tests = 0;

  m_put_string((uint8_t *)"<registry name=\"");
  m_put_string((uint8_t *)registry_name);
  m_put_string((uint8_t *)"\">\r\n");
  m_put_string((uint8_t *)"  <file>");
  m_put_string((uint8_t *)file_name);
  m_put_string((uint8_t *)"</file>\r\n");
}

/** @brief Implementation of the assertion interface callback.
  */
void assert_pre_construct_callback(void)
{
}

/*
  @note The assertion interface callback is used to complete open XML tags in the
  same way as would happen if a cu_assertion occurs, and to halt the system
  according to the expected way the CUnit framework will behave.  It is
  important that cu_exit() be called instead of allowing the system ASSERT
  to halt the system because the simulator is configured to recognise cu_exit();
*/
void assert_pre_destruct_callback(void)
{
  cu_registry_destruct();
  cu_exit();
}

void cu_registry_destruct(void)
{
    /*Close open tags as ASSERT shall end the simulation.
    *Note that the call hierarchy in CUnit is
    *
    *                      Test Registry
    *                            |
    *             ------------------------------
    *             |                            |
    *          Suite '1'      . . . .       Suite 'N'
    *             |                            |
    *       ---------------             ---------------
    *       |             |             |             |
    *    Test '11' ... Test '1M'     Test 'N1' ... Test 'NM'
    */

    cu_suite_destruct();

    if (m_cu_summary_enabled)
    {
        char buffer[100];
        sprintf(buffer,
              "<!-- %u tests - %u passed - %u failed - (total of %u failures) -->\r\n ",
              m_num_of_tests,
              m_num_of_passed_tests,
              m_num_of_failed_tests,
              (uint16_t)error_num);

        m_put_string((uint8_t *)buffer);
    }

    if(m_cu_reg_running)
    {
        m_put_string((uint8_t *)"</registry>\r\n ");
        m_cu_reg_running = false;
    }
}

void cu_suite_construct(uint8_t const * const suite_name)
{
  m_cu_suite_running = true;
  m_put_string((uint8_t *)"  <suite name=\"");
  m_put_string(suite_name);
  m_put_string((uint8_t *)"\">\r\n");
  cu_test_setup    = cu_setup_func_default;
  cu_test_teardown = cu_teardown_func_default;
}

void cu_suite_destruct(void)
{
  cu_test_destruct();
  if(m_cu_suite_running)
  {
    m_put_string((uint8_t *)"  </suite>\r\n ");
    m_cu_suite_running = false;
  }
}

void cu_test_construct(uint8_t const * const test_name)
{
  m_cu_test_running = true;
  m_cu_test_failed = false;
  m_num_of_tests++;

  m_put_string((uint8_t *)"    <test name=\"");
  m_put_string(test_name);
  m_put_string((uint8_t *)"\">\r\n");
}

void cu_test_destruct(void)
{
  if(m_cu_test_running)
  {
    m_put_string((uint8_t *)"    </test>\r\n");

    if (m_cu_test_failed)
    {
        m_num_of_failed_tests++;
    }
    else
    {
        m_num_of_passed_tests++;
    }

    m_cu_test_running = false;
  }


}

void cu_exit(void)
{
#ifdef WIN32
  exit(error_num == 0 ? 0 : 1);
#else
  while (true)
  {
  }
#endif
}

void cu_assert_handler(const uint8_t * p_file, uint16_t line_num, const uint8_t * p_expr, cu_severity_t severity)
{
  error_num++;
  m_cu_test_failed = true;

  m_put_string((uint8_t *)"      <error number=\"");
  m_put_decword(error_num);
  m_put_string((uint8_t *)"\" fatal=\"");
  m_put_string((const uint8_t *)(severity == CU_FATAL ? "true" : "false"));
  m_put_string((uint8_t *)"\">\r\n");
  if (p_file)
  {
    m_put_string((uint8_t *)"        <file>");
    m_put_string(p_file);
    m_put_string((uint8_t *)"</file>\r\n");
  }
  m_put_string((uint8_t *)"        <line>");
  m_put_decword(line_num);
  m_put_string((uint8_t *)"</line>\r\n");
  if (p_expr)
  {
    m_put_string((uint8_t *)"        <expression>");
    m_put_string(p_expr);
    m_put_string((uint8_t *)"</expression>\r\n");
  }
  m_put_string((uint8_t *)"      </error>\r\n");
  if (severity == CU_FATAL)
  {
    assert_pre_destruct_callback();
  }
}

void cu_print_string(const uint8_t * p_buffer)
{
    char str[110];
    strcpy(str, "      <!-- ");
    strcat(str, (const char *)p_buffer);
    strcat(str, " -->\r\n");
    m_put_string((uint8_t *)str);
}


void cu_assert(bool expr, const uint8_t * p_file, uint16_t line_num, const uint8_t * p_message)
{
    if(expr != true)
    {
        cu_assert_handler(p_file, line_num, (const uint8_t*)(p_message), CU_NONFATAL);
    }
}


void cu_assert_int_equal(uint32_t actual, uint32_t expected, const uint8_t * p_file, uint16_t line_num)
{
    if((actual) != (expected))
    {
        char buffer[100]; \
        sprintf(buffer, "CU_ASSERT_INT_EQUAL(%lu,%lu)", (unsigned long)actual, (unsigned long)expected); \
        cu_assert_handler(p_file, line_num, (const uint8_t*)buffer, CU_NONFATAL);
    }
}

void cu_enable_summary(void)
{
    m_cu_summary_enabled = true;
}
