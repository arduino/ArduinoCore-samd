/*
 Copyright (c) 2011 Arduino.  All right reserved.

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

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

// static int _readResolution = 10;
// static int _writeResolution = 8;

// void analogReadResolution(int res) {
	// _readResolution = res;
// }

// void analogWriteResolution(int res) {
	// _writeResolution = res;
// }

// static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	// if (from == to)
		// return value;
	// if (from > to)
		// return value >> (from-to);
	// else
		// return value << (to-from);
// }

// eAnalogReference analog_reference = AR_DEFAULT;

// void analogReference(eAnalogReference ulMode)
// {
	// analog_reference = ulMode;
// }

// uint32_t analogRead(uint32_t ulPin)
// {
  // uint32_t ulValue = 0;
  // uint32_t ulChannel;

  // if (ulPin < A0)
    // ulPin += A0;

  // ulChannel = g_APinDescription[ulPin].ulADCChannelNumber ;

// #if defined __SAM3U4E__
	// switch ( g_APinDescription[ulPin].ulAnalogChannel )
	// {
		// // Handling ADC 10 bits channels
		// case ADC0 :
		// case ADC1 :
		// case ADC2 :
		// case ADC3 :
		// case ADC4 :
		// case ADC5 :
		// case ADC6 :
		// case ADC7 :
			// // Enable the corresponding channel
			// adc_enable_channel( ADC, ulChannel );

			// // Start the ADC
			// adc_start( ADC );

			// // Wait for end of conversion
			// while ((adc_get_status(ADC) & ADC_SR_DRDY) != ADC_SR_DRDY)
				// ;

			// // Read the value
			// ulValue = adc_get_latest_value(ADC);
			// ulValue = mapResolution(ulValue, 10, _readResolution);

			// // Disable the corresponding channel
			// adc_disable_channel( ADC, ulChannel );

			// // Stop the ADC
			// //      adc_stop( ADC ) ; // never do adc_stop() else we have to reconfigure the ADC each time
			// break;

		// // Handling ADC 12 bits channels
		// case ADC8 :
		// case ADC9 :
		// case ADC10 :
		// case ADC11 :
		// case ADC12 :
		// case ADC13 :
		// case ADC14 :
		// case ADC15 :
			// // Enable the corresponding channel
			// adc12b_enable_channel( ADC12B, ulChannel );

			// // Start the ADC12B
			// adc12b_start( ADC12B );

			// // Wait for end of conversion
			// while ((adc12b_get_status(ADC12B) & ADC12B_SR_DRDY) != ADC12B_SR_DRDY)
				// ;

			// // Read the value
			// ulValue = adc12b_get_latest_value(ADC12B) >> 2;
			// ulValue = mapResolution(ulValue, 12, _readResolution);

			// // Stop the ADC12B
			// //      adc12_stop( ADC12B ) ; // never do adc12_stop() else we have to reconfigure the ADC12B each time

			// // Disable the corresponding channel
			// adc12b_disable_channel( ADC12B, ulChannel );
			// break;

		// // Compiler could yell because we don't handle DAC pins
		// default :
			// ulValue=0;
			// break;
	// }
// #endif

// #if defined __SAM3X8E__ || defined __SAM3X8H__
	// static uint32_t latestSelectedChannel = -1;
	// switch ( g_APinDescription[ulPin].ulAnalogChannel )
	// {
		// // Handling ADC 12 bits channels
		// case ADC0 :
		// case ADC1 :
		// case ADC2 :
		// case ADC3 :
		// case ADC4 :
		// case ADC5 :
		// case ADC6 :
		// case ADC7 :
		// case ADC8 :
		// case ADC9 :
		// case ADC10 :
		// case ADC11 :

			// // Enable the corresponding channel
			// if (ulChannel != latestSelectedChannel) {
				// adc_enable_channel( ADC, ulChannel );
				// if ( latestSelectedChannel != -1 )
					// adc_disable_channel( ADC, latestSelectedChannel );
				// latestSelectedChannel = ulChannel;
			// }

			// // Start the ADC
			// adc_start( ADC );

			// // Wait for end of conversion
			// while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY)
				// ;

			// // Read the value
			// ulValue = adc_get_latest_value(ADC);
			// ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);

			// break;

		// // Compiler could yell because we don't handle DAC pins
		// default :
			// ulValue=0;
			// break;
	// }
// #endif

	// return ulValue;
// }


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.

void analogWrite(uint32_t ulPin, uint32_t ulValue) {

	 uint32_t attr = g_APinDescription[ulPin].ulPinAttribute;
	 uint32_t pwm_name = g_APinDescription[ulPin].ulTCChannel;
	 uint8_t isTC = 0; 
	 uint8_t Channelx;
	 Tc* TCx;
	 Tcc* TCCx;

	// if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG) {
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
	// }

	if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM) {
	 
	 if (g_APinDescription[ulPin].ulPinType == PIO_TIMER)
	 {
		 // Set selected Pin as TC/TCC Waveform out (PMUX : E ) 
		 if(g_APinDescription[ulPin].ulPin <= 15)
		 {
			PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = (uint32_t)(PORT_WRCONFIG_WRPINCFG |PORT_WRCONFIG_WRPMUX| 1 << (g_APinDescription[ulPin].ulPin)|(PORT_WRCONFIG_PMUXEN)|(0x4 << PORT_WRCONFIG_PMUX_Pos) );
		 } else {
		    PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = (uint32_t)(PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX| PORT_WRCONFIG_HWSEL| 1 << (g_APinDescription[ulPin].ulPin - 16)|(PORT_WRCONFIG_PMUXEN)|(0x4 << PORT_WRCONFIG_PMUX_Pos) );
		 }
	 }
	 
	 if (g_APinDescription[ulPin].ulPinType == PIO_TIMER_ALT)
	 {
		  // Set selected Pin as TC/TCC Waveform out (PMUX : F )
		  if(g_APinDescription[ulPin].ulPin <= 15)
		  {
			PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = (uint32_t)(PORT_WRCONFIG_WRPINCFG |PORT_WRCONFIG_WRPMUX| 1 << (g_APinDescription[ulPin].ulPin)|(PORT_WRCONFIG_PMUXEN)|(0x5 << PORT_WRCONFIG_PMUX_Pos) );
		  } else {
			PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = (uint32_t)(PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX| PORT_WRCONFIG_HWSEL| 1 << (g_APinDescription[ulPin].ulPin - 16)|(PORT_WRCONFIG_PMUXEN)|(0x5 << PORT_WRCONFIG_PMUX_Pos) );
		  }
	  }
	 
	 
		switch (g_APinDescription[ulPin].ulPWMChannel) 
		{
			case  PWM3_CH0 :
			TCx = TC3;
			Channelx = 0;
			isTC = 1;
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
		
		// --Set PORT  
		
		if (isTC)
		{
			// -- Enable clocks according to TCCx instance to use
			 switch ((uint32_t) TCx)
			{
				case (uint32_t) TC3 :
				//Enable TCx Bus clock (Timer counter control clock)
				PM->APBCMASK.reg |= PM_APBCMASK_TC3;
				//Enable GCLK for TC3 (timer counter input clock)
				GCLK->CLKCTRL.reg = (uint16_t) ((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ( 0x1B << GCLK_CLKCTRL_ID_Pos)));
				break;
			
				case (uint32_t) TC4 :
				//Enable TCx Bus clock (Timer counter control clock)
				PM->APBCMASK.reg |= PM_APBCMASK_TC4;
				//Enable GCLK for TC4 (timer counter input clock)
				GCLK->CLKCTRL.reg = (uint16_t) ((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ( 0x1C << GCLK_CLKCTRL_ID_Pos)));
				break;
			
				case (uint32_t) TC5 :
				//Enable TCx Bus clock (Timer counter control clock)
				PM->APBCMASK.reg |= PM_APBCMASK_TC5;
				//Enable GCLK for TC5 (timer counter input clock)
				GCLK->CLKCTRL.reg = (uint16_t) ((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ( 0x1C << GCLK_CLKCTRL_ID_Pos)));
				break;
			
			}
			
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
		
		} else {
		
		// -- Enable clocks according to TCCx instance to use
			 switch ((uint32_t) TCCx)
			 {
				case (uint32_t) TCC0 :
				//Enable TCC0 Bus clock (Timer counter control clock)
				PM->APBCMASK.reg |= PM_APBCMASK_TCC0;
				//Enable GCLK for TCC0 (timer counter input clock)
				GCLK->CLKCTRL.reg = (uint16_t) ((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ( 0x1A << GCLK_CLKCTRL_ID_Pos)));
				break;
				
				case (uint32_t) TCC1 :
				//Enable TCC1 Bus clock (Timer counter control clock)
				PM->APBCMASK.reg |= PM_APBCMASK_TCC1;
				//Enable GCLK for TCC1 (timer counter input clock)
				GCLK->CLKCTRL.reg = (uint16_t) ((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ( 0x1A << GCLK_CLKCTRL_ID_Pos)));
				break;
				
				case (uint32_t) TCC2 :
				//Enable TCC2 Bus clock (Timer counter control clock)
				PM->APBCMASK.reg |= PM_APBCMASK_TCC2;
				//Enable GCLK for TCC2 (timer counter input clock)
				GCLK->CLKCTRL.reg = (uint16_t) ((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ( 0x1B << GCLK_CLKCTRL_ID_Pos)));
				break;
			}
			
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
		return;
	}

	// -- Defaults to digital write
	pinMode(ulPin, OUTPUT);
	//ulValue = mapResolution(ulValue, _writeResolution, 8);
	if (ulValue < 128)
		digitalWrite(ulPin, LOW);
	else
		digitalWrite(ulPin, HIGH);
}

#ifdef __cplusplus
}
#endif
