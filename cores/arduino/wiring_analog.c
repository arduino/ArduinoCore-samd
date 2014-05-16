/*
 Copyright (c) 2014 Arduino.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "wiring_analog.h"
#include "wiring_digital.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t analogRead( uint32_t ulPin )
{
  uint32_t ulValue = 0 ;
/*
  uint32_t ulChannel ;

  ulChannel = g_APinDescription[ulPin].ulADCChannelNumber ;

  static uint32_t latestSelectedChannel = -1;

  switch ( g_APinDescription[ulPin].ulAnalogChannel )
  {
    // Handling ADC 12 bits channels
    case ADC0 :
    case ADC1 :
    case ADC2 :
    case ADC3 :
    case ADC4 :
    case ADC5 :
    break;
    // Compiler could yell because we don't handle DAC pin
    default :
      ulValue=0;
    break;
  }
*/

  return ulValue;
 }


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite( uint32_t ulPin, uint32_t ulValue )
 {
   uint32_t attr = g_APinDescription[ulPin].ulPinAttribute ;
//   uint32_t pwm_name = g_APinDescription[ulPin].ulTCChannel ;
   uint8_t isTC = 0 ;
   uint8_t Channelx ;
   Tc* TCx ;
   Tcc* TCCx ;

   if ( (attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG )
   {
    // EAnalogChannel channel = g_APinDescription[ulPin].ulADCChannelNumber;
    // if (channel == DA0 || channel == DA1) {
      // uint32_t chDACC = ((channel == DA0) ? 0 : 1);
      // if (dacc_get_channel_status(DACC_INTERFACE) == 0) {
        // /* Enable clock for DACC_INTERFACE */
        // pmc_enable_periph_clk(DACC_INTERFACE_ID);

        // /* Reset DACC registers */
        // dacc_reset(DACC_INTERFACE);

        // /* Half word transfer mode */
        // dacc_set_transfer_mode(DACC_INTERFACE, 0);

        // /* Power save:
         // * sleep mode  - 0 (disabled)
         // * fast wakeup - 0 (disabled)
         // */
        // dacc_set_power_save(DACC_INTERFACE, 0, 0);
        // /* Timing:
         // * refresh        - 0x08 (1024*8 dacc clocks)
         // * max speed mode -    0 (disabled)
         // * startup time   - 0x10 (1024 dacc clocks)
         // */
        // dacc_set_timing(DACC_INTERFACE, 0x08, 0, 0x10);

        // /* Set up analog current */
        // dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02) |
                      // DACC_ACR_IBCTLCH1(0x02) |
                      // DACC_ACR_IBCTLDACCORE(0x01));
      // }

      // /* Disable TAG and select output channel chDACC */
      // dacc_set_channel_selection(DACC_INTERFACE, chDACC);

      // if ((dacc_get_channel_status(DACC_INTERFACE) & (1 << chDACC)) == 0) {
        // dacc_enable_channel(DACC_INTERFACE, chDACC);
      // }

      // // Write user value
      // ulValue = mapResolution(ulValue, _writeResolution, DACC_RESOLUTION);
      // dacc_write_conversion_data(DACC_INTERFACE, ulValue);
      // while ((dacc_get_interrupt_status(DACC_INTERFACE) & DACC_ISR_EOC) == 0);
      // return;
    // }
  }

  if ( (attr & PIN_ATTR_PWM) == PIN_ATTR_PWM )
  {
    if ( (g_APinDescription[ulPin].ulPinType == PIO_TIMER) || g_APinDescription[ulPin].ulPinType == PIO_TIMER_ALT )
    {
      pinPeripheral( ulPin, g_APinDescription[ulPin].ulPinType ) ;
    }

    Channelx = GetTCChannelNumber( g_APinDescription[ulPin].ulPWMChannel ) ;
    if ( GetTCChannelNumber( g_APinDescription[ulPin].ulPWMChannel ) >= TCC_INST_NUM )
    {
      isTC = 1 ;
      TCx = (Tc*) GetTC( g_APinDescription[ulPin].ulPWMChannel ) ;
    }
    else
    {
      isTC = 0 ;
      TCCx = (Tcc*) GetTC( g_APinDescription[ulPin].ulPWMChannel ) ;
    }

/*
    switch ( g_APinDescription[ulPin].ulPWMChannel )
    {
      case PWM3_CH0 :
        TCx = TC3 ;
        Channelx = 0 ;
        isTC = 1 ;
      break;

      case  PWM3_CH1:
      TCx = TC3 ;
      Channelx = 1;
      isTC = 1;
      break;

      case  PWM0_CH0 :
      TCCx = TCC0;
      Channelx = 0;
      break;

      case  PWM0_CH1 :
      TCCx = TCC0;
      Channelx = 1;
      break;

      case  PWM0_CH4 :
      TCCx = TCC0;
      //Channelx = 4;
      Channelx = 0;
      break;

      case  PWM0_CH5 :
      TCCx = TCC0;
      //Channelx = 5;
      Channelx = 1;
      break;

      case  PWM0_CH6 :
      TCCx = TCC0;
      //Channelx = 6;
      Channelx = 2;
      break;

      case  PWM0_CH7 :
      TCCx = TCC0;
      //Channelx = 7;
      Channelx = 3;
      break;

      case  PWM1_CH0 :
      TCCx = TCC1;
      Channelx = 0;
      break;

      case  PWM1_CH1 :
      TCCx = TCC1;
      Channelx = 1;
      break;

      case  PWM2_CH0 :
      TCCx = TCC2;
      Channelx = 0;
      break;

      case  PWM2_CH1 :
      TCCx = TCC2;
      Channelx = 1;
      break;
    }
*/

    // Enable clocks according to TCCx instance to use
    switch ( GetTCNumber( g_APinDescription[ulPin].ulPWMChannel ) )
    {
      case 0: // TCC0
        //Enable GCLK for TCC0 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;
      break ;

      case 1: // TCC1
        //Enable GCLK for TCC1 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;
      break ;

      case 2: // TCC2
        //Enable GCLK for TCC2 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 )) ;
      break ;

      case 3: // TC3
        //Enable GCLK for TC3 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 ));
      break ;

      case 4: // TC4
        //Enable GCLK for TC4 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 ));
      break ;

      case 5: // TC5
        //Enable GCLK for TC5 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 )) ;
      break ;
    }

    // Set PORT
    if ( isTC )
    {
      // -- Configure TC
      //DISABLE TCx
      TCx->COUNT8.CTRLA.reg &=~(TC_CTRLA_ENABLE);
      //Set Timer counter Mode to 8 bits
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8;
      //Set TCx as normal PWM
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
      //Set TCx in waveform mode Normal PWM
      TCx->COUNT8.CC[Channelx].reg = (uint8_t) ulValue;
      //Set PER to maximum counter value (resolution : 0xFF)
      TCx->COUNT8.PER.reg = 0xFF;
      // Enable TCx
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    }
    else
    {
      // -- Configure TCC

      //DISABLE TCCx
      TCCx->CTRLA.reg &=~(TCC_CTRLA_ENABLE);
      //Set TCx as normal PWM
      TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
      //Set TCx in waveform mode Normal PWM
      TCCx->CC[Channelx].reg = (uint32_t)ulValue;
      //Set PER to maximum counter value (resolution : 0xFF)
      TCCx->PER.reg = 0xFF;
      //ENABLE TCCx
      TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
    }

    return ;
  }

  // -- Defaults to digital write
  pinMode( ulPin, OUTPUT ) ;

  //ulValue = mapResolution(ulValue, _writeResolution, 8);

  if ( ulValue < 128 )
  {
    digitalWrite( ulPin, LOW ) ;
  }
  else
  {
    digitalWrite( ulPin, HIGH ) ;
  }
}

#ifdef __cplusplus
}
#endif
