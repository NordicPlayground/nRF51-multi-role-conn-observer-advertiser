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
 * @author Joar Rusten
 * @author Asbjørn Sæbø
 * @author Knut Auvor Grythe
 *
 * cUnit framework for embedded devices.
 *
 * @defgroup cunit cUnit
 * @{
 */

//lint -epuc  lint seems to assume that "" strings are char, not unsigned char

#ifndef CUNIT_H_
#define CUNIT_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

typedef enum
{
  CU_NONFATAL,
  CU_FATAL
} cu_severity_t;

typedef void (*cu_test_setup_function_t)(void);
typedef void (*cu_test_teardown_function_t)(void);

/**
 * Assert macro.
 *
 * @param expr Expression that must evaluate true
 */
#define CU_ASSERT(expr)\
/*lint save -e506,-e774 Constant value Boolean, Boolean within 'if' always evaluates to True */ \
{\
    cu_assert(expr, (const uint8_t*)__FILE__, (uint16_t)__LINE__, (const uint8_t*)"CU_ASSERT( "# expr " )");\
}
//lint -restore 

/** Asserts that actual == expected.
 * Reports failure and causes test to abort.
 */
#define CU_ASSERT_INT_EQUAL(actual, expected)\
{\
    cu_assert_int_equal(actual, expected, (const uint8_t*)__FILE__, (uint16_t)__LINE__);\
}

#define CU_MESSAGE(...)\
{\
  char buffer[100];\
  sprintf(buffer, __VA_ARGS__);\
  cu_print_string((const uint8_t *)buffer); \
}
/**
 * Start tag for the test registry.
 * This tag must be used in the beginning of the body of the test registry.
 * Only one test registry can be present in each test file.
 */
#define CU_REGISTRY_START(registry_name) \
int main(void)\
{\
cu_registry_construct(# registry_name, __FILE__);\
/* __FILE__ must be use here in the cunit header file (as opposed to the cunit implementation) */
/* so that the resolved name is that of the calling c-file, not that of the cunit.c file. */ 

/**
 * Start tag for the standalone test registry.
 * This tag must be used in the beginning of the body of the test registry.
 * Multiple standalone registries may be present in the same test file.
 *
 * When using this registry type a dedicated main function must be added
 * in the tests main.c file.
 */
#define CU_REGISTRY_START_STANDALONE(registry_name) \
static int registry_name(void)\
{\
cu_registry_construct((const uint8_t*)(# registry_name), (const uint8_t*)(__FILE__));\
/* __FILE__ must be use here in the cunit header file (as opposed to the cunit implementation) */
/* so that the resolved name is that of the calling c-file, not that of the cunit.c file. */ 




/** End tag of the test registry.
 * This tag must be used in the end of the body of the test registry .
 */
#define CU_REGISTRY_END \
cu_registry_destruct();\
cu_exit();\
return 0;\
}

#define CU_SUITE_SETUP_FUNC(setup_func)\
cu_set_suite_setup_func(setup_func);\

#define CU_SUITE_TEARDOWN_FUNC(teardown_func)\
cu_set_suite_teardown_func(teardown_func);\

/** Start tag for a test suite.
 * This tag must be used in the beginning of the body of each test suite.
 * Several test suites can be present in each test file.
 *
 * @param suite_name Unique name for the test suite in the test registry.
 */
#define CU_SUITE_START(suite_name)\
static void suite_name(void)\
{\
cu_suite_construct((const uint8_t*)(# suite_name));\

/** Start tag for a test suite which is public (non static).
 * This tag must be used in the beginning of the body of each test suite.
 * Several test suites can be present in each test file.
 *
 * @param suite_name Unique name for the test suite in the test registry.
 */
#define CU_SUITE_START_PUBLIC(suite_name)\
void suite_name(void)\
{\
cu_suite_construct((const uint8_t*)(# suite_name));\

/** End tag for a test suite.
 * This tag must be used in end of the body of each test suite.
 */
#define CU_SUITE_END \
cu_suite_destruct(); \
}\

/** Start tag for a test.
 * This tag must be used in the beginning of the body of each test.
 * 
 * @param test_name Unique name for test in the test registry.
 */                  
#define CU_TEST_START(test_name)\
static void test_name(void)\
{\
cu_call_setup_func();\
cu_test_construct((const uint8_t*)(# test_name));\

/** End tag for a test.
 * This tag must be used in the end of the body of each test.
 */
#define CU_TEST_END \
cu_test_destruct(); \
cu_call_teardown_func(); \
}\

/** Macro to add a registry.
 *
 * @param func Name of registry to be added.
 */
#define CU_ADD_REGISTRY(func) \
(void)func()

/** Macro to add a test to a test suite.
 *
 * @param func Name of test to be added.
 */
#define CU_SUITE_ADD_TEST(func) \
func()

/** Macro to add a test suite to the test registry.
 *
 * @param func Name of test suite to be added.
 */
#define CU_REGISTRY_ADD_SUITE(func) \
func()

/** Macro to add a extern test suite to the test registry.
 *
 * @param func Name of test suite to be added.
 */
#define CU_REGISTRY_ADD_SUITE_EXTERN(func) \
{ \
    extern void func(void); \
    func(); \
}

#define CU_ENABLE_SUMMARY()\
cu_enable_summary()

void cu_registry_construct(uint8_t const * const registry_name, uint8_t const  * const file_name);

void cu_registry_destruct(void);

void cu_suite_construct(uint8_t const * const suite_name);

void cu_suite_destruct(void);

void cu_test_construct(uint8_t const * const test_name);

void cu_test_destruct(void);

void cu_exit(void);

/** Assert handler for cUnit assertions.
 * This function is used indirectly via cUnit assert macros to
 * print information about the current error.
 *
 * This function will generate the following XML output:
 *
 * <error number="x" fatal="false">
 *  <file> file.c </file>
 *  <line> y </line>
 *  <expression> expr </expression> 
 * </error>
 *
 * Where x denotes the incremental number of the current error message, y denotes
 * the line in the source file of the test where the error was detected and expr
 * denotes the expression that failed. The <file> and <expression> tags will be omitted
 * if no information is provided.
 *
 * @param p_file    Pointer to the file name where the assertions occured (or NULL)
 * @param line_num  Line number where assertion is located.
 * @param p_expr    Pointer to message string to be printed in the expression field (or NULL)
 * @param severity  Set to CU_FATAL to make the application halt if the assertion fails, CU_NONFATAL otherwise
 *
 */
void cu_assert_handler(const uint8_t * p_file, uint16_t line_num, const uint8_t * p_expr, cu_severity_t severity);

void cu_print_string(const uint8_t * p_buffer);

void cu_set_suite_setup_func(cu_test_setup_function_t func);

void cu_call_setup_func(void);

void cu_set_suite_teardown_func(cu_test_teardown_function_t func);

void cu_call_teardown_func(void);

void cu_assert(bool expr, const uint8_t * p_file, uint16_t line_num, const uint8_t *p_message);

void cu_assert_int_equal(uint32_t actual, uint32_t expected, const uint8_t * p_file, uint16_t line_num);

void cu_enable_summary(void);

#endif

/** @} */

/** @page page_cunit cUnit
  This cUnit implementation is designed for small embedded systems based on the
  8051 architecture. The cUnit framework does not use function pointers or
  dynamic memory allocation.

  This cUnit framework supports the following features:

  - Test registry
  - Test suite
  - Test
  - Assertion
  - XML output to terminal via UART

  A test environment is implemented in a single c source file that contains one test registry,
  one or several test suites and one or several tests.

  All tests that are to be executed must be registered in one or several of the test suites, 
  which in turn must be registered in the test registry.

  The following code snippet demonstrates the use of the cUnit test framework.

  @code

   #include <stdbool.h>     // Include std lib.
   #include "cunit.h"       // Include cUnit framework.

  // To define a test with the name "test_1", use the CU_TEST_START tag. 
  // The name you specify here will eventually become the name of a function.
  // I.e. use the same naming convention for tests as for functions. 
  CU_TEST_START( test_1 )    
  {
    //.. Here you can add whatever code you need to perform your test.
    CU_ASSERT( <expression that should be true> )      // Test will pass when expression is indeed true
    CU_ASSERT( false );                                // Force an assertion (test will fail).
  }
  CU_TEST_END // Use the CU_TES_END tag to mark the end of the test.

  // To define a test with the name "suite_1", use the CU_SUITE_START tag. 
  // The name you specify here will eventually become the name of a function.
  // I.e. use the same naming convention for tests as for functions. 
  CU_SUITE_START( suite_1 )
  {
    CU_SUITE_ADD_TEST( test_1 );  // Register test "test_1" in suite "suite_1"
  }
  CU_SUITE_END // Use the CU_SUITE_END tag to mark the end of a suite.

  // To define a test registry with the name "registry", use the 
  // CU_TEST_REGISTRY_START tag. 
  CU_REGISTRY_START( registry )
  {
    CU_REGISTRY_ADD_SUITE( suite_1 );  // Register suite "suite_1" in test registry "registry".
  }
  CU_REGISTRY_END  // Use the CU_TEST_REGISTRY_END tag to mark the end of the test registry.

  @endcode

  The order in which the different elements are written in the source file is important.

  All the tests must be located in the beginning of the source file followed by the test
  suites which in turn must be followed by the test registry.


  This test framework is designed for the Keil simulator environment, the following script
  can be used to automate testing. 
  
  @code
    SLOG>serial.log
    G,assert_destruct
    SLOG OFF
    LOG>coverage.log
    COVERAGE
    LOG OFF
    EXIT
    EXIT
  @endcode

  Save this script in a *.ini file and register the file in the Kiel project file
  under @htmlonly "Project" &rarr; "Options for Target" &rarr; "Debug" &rarr; "Initialization file".
  @endhtmlonly

  When this script is used in conjunction with the cUnit framework and the assertion module,
  the simulator will automatically execute all tests that are registered in the test 
  registry. All assertions that are generated by the tests or by modules under test will be
  logged in the <b>serial.log</b> file. Code coverage will be logged in the <b>coverage.log</b>
  file.

  The following command can be used to invoke the Keil simulator from the command line etc.

  <kpath>/uv3.exe -d <ppath>/<pname>

  Here <b><kpath></b> denotes the path to your Keil installation, <b><ppath></b> is the
  path to your Keil project and <b><pname></b> is the name of the project
  you want to execute.

*/

