/*
 *  Copyright (c) 2018 MattairTech LLC. All right reserved.
 * 
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 * 
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU Lesser General Public License for more details.
 * 
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

/* Currently, there are only two options in the menu, to enable or disable config.h.
 * When config.h is enabled, comment out lines to disable individual options.
 */
#if defined(CONFIG_H_ENABLED)

  /* The standard PinDescription table uses 12 bytes per pin. Define PIN_DESCRIPTION_TABLE_SIMPLE
   * to use a more compact format that uses only 4 bytes per pin (currently only available
   * for the D11 chips). In this case, the PinType, PinAttribute, and GCLKCCL columns are not used
   * (they are not required). Additionally, the SetPortPin() and SetExtIntADC() macros are used to
   * pack Port and Pin into the PortPin column, and ExtInt and ADCChannelNumber into the ExtIntADC
   * column. See the variant README.md for more information. Note that external libraries that
   * reference the PinDescription table directly (uncommon) will no longer work. This define can be 
   * combined with the PIN_MAP_COMPACT define, which is available in variant.h of the D11 variants.
   * This can save from 10's to over 250 bytes.
   *
   * Note that when using this define, EXTERNAL_INT_15 and the EXTERNAL_INT_NMI are NOT available
   * and only ADC_Channel0 through ADC_Channel14 are available.
   */
  //#define PIN_DESCRIPTION_TABLE_SIMPLE

  /* Define PIN_PERIPHERAL_CHECKS_DISABLED to disable some sanity and other checks at the beginning 
   * of pinPeripheral() (ie: out of bounds access, pin isn't usable as PIO, verify hardware peripheral
   * requested actually exists on pin). This saves about 100 bytes.
   */
  #define PIN_PERIPHERAL_CHECKS_DISABLED

  /* Define ADC_NO_INIT_IF_UNUSED and/or REF_NO_INIT_IF_UNUSED to save the code space used by the
   * initialization functions of the ADC, DAC, and REF, if they are not used by the core. Do not
   * set this define if an external library is used for the ADC, DAC, or REF, as that library probably
   * expects the associated hardware to be already initialized. Note that if using a core function
   * (ie: analogWrite()), initialization will still occur automatically. This saves about 350 bytes
   * with ADC_NO_INIT_IF_UNUSED and 50 bytes with DAC_NO_INIT_IF_UNUSED.
   */
  #define ADC_NO_INIT_IF_UNUSED
  #define DAC_NO_INIT_IF_UNUSED

  /* Define DISABLE_ADC_CALIBRATION to disable ADC calibration using the NVM factory values, which
   * occurs during initialization, and save a few bytes. This setting is not generally recommended.
   */
  //#define DISABLE_ADC_CALIBRATION

  /* Define TRUST_RESET_DEFAULTS to avoid writing default values into registers during hardware
   * initialization, saving about 50 bytes (currently).
   */
  #define TRUST_RESET_DEFAULTS

  /* Define NO_ADDITIONAL_GCLKS to disable initialization of GENERIC_CLOCK_GENERATOR_192MHz (D51) and
   * GENERIC_CLOCK_GENERATOR_48MHz (all others), which are currently unused in the core. This currently
   * saves about 48 bytes.
   */
  #define NO_ADDITIONAL_GCLKS

  /* Define NO_OSC_HS_GCLK to disable initialization of GENERIC_CLOCK_GENERATOR_OSC_HS, which is setup
   * to run at 8MHz with the D21, D11, and L21, but is unused by the core. This saves about 30 bytes.
   */
  #define NO_OSC_HS_GCLK

  /* Define NO_DELAY_HIGH_WORD to disable the high word variable (_ulTickCountHighWord) in delay.c,
   * which is used to allow counting beyond the 50-day maximum (for a 32-bit variable counting in
   * milliseconds). This saves about 64 bytes.
   */
  #define NO_DELAY_HIGH_WORD

  /* Define LONG_LONG_PRINT_FLOAT to enable the use of Unsigned Long Long variables for printing
   * the integer part of single or double floating point numbers using the Print class. This allows
   * printing integers up to +/-18,446,744,073,709,551,615, rather than the default +/-4,294,967,295.
   * This cost a little code space.
   */
  //#define LONG_LONG_PRINT_FLOAT
#endif

#endif // _CONFIG_H_
