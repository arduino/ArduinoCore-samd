


#ifndef USART_SETUP_H_
#define USART_SETUP_H_
/** \name Embedded debugger SOFIA SERCOM interface definitions
* @{
*/
#define EDBG_SOFIA_MODULE              SERCOM5
#define EDBG_SOFIA_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3//USART_RX_3_TX_2_XCK_3
#define EDBG_SOFIA_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_SOFIA_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_SOFIA_SERCOM_PINMUX_PAD2  PINMUX_PB22D_SERCOM5_PAD2
#define EDBG_SOFIA_SERCOM_PINMUX_PAD3  PINMUX_PB23D_SERCOM5_PAD3
#define EDBG_SOFIA_SERCOM_DMAC_ID_TX   SERCOM5_DMAC_ID_TX
#define EDBG_SOFIA_SERCOM_DMAC_ID_RX   SERCOM5_DMAC_ID_RX
/** @} */






struct usart_module usart_instance;
void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	//! [setup_change_config]
	config_usart.baudrate    = 57600;
	config_usart.mux_setting = EDBG_SOFIA_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_SOFIA_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_SOFIA_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_SOFIA_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_SOFIA_SERCOM_PINMUX_PAD3;
	//config_usart.start_frame_detection_enable = true;
	//! [setup_change_config]
	while (usart_init(&usart_instance, SERCOM5, &config_usart) != STATUS_OK) {
		
	}
	usart_enable(&usart_instance);
}

#endif /* USART_SETUP_H_ */


