/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 249 $
 */
 
/**
A template file for system_device.c is provided by ARM but adapted by the 
silicon vendor to match their actual device. As a minimum requirement this file 
must provide a device specific system configuration function and a global 
variable that contains the system frequency. It configures the device and 
initializes typically the oscillator (PLL) that is part of the microcontroller 
device.
*/

#include <stdint.h>
#include "nrf.h"

/**
  Sets up the RAM.

  This function configures the RAM. This function must be the first function called.
*/
void RamInit (void)
{
}

/**
  Sets up the microcontroller system.

  Typically this function configures the oscillator (PLL) that is part of the
  microcontroller device. For systems with variable clock speed it also updates
  the variable SystemCoreClock. SystemInit is called from startup_device file
  before entering main() function.
*/
void SystemInit (void) 
{
}

/**
  Updates the variable SystemCoreClock and must be called whenever the core
  clock is changed during program execution.
  
  SystemCoreClockUpdate() evaluates the clock register settings and calculates
  the current core clock.
*/
void SystemCoreClockUpdate (void)
{
}

/*lint --flb "Leave library region" */
