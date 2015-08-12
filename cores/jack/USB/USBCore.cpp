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

#include "../Arduino.h"
#include "USBCore.h"
#include "USB/USB_device.h"   // needed for USB PID define
#include "USBDesc.h"
#include "USBAPI.h"

//#define TRACE_CORE(x)	x
#define TRACE_CORE(x)

static char isRemoteWakeUpEnabled = 0;
static char isEndpointHalt = 0;

const uint16_t STRING_LANGUAGE[2] = {
	(3<<8) | (2+2),
	0x0409	// English
};

#ifndef USB_PRODUCT
// If no product is provided, use USB IO Board
#define USB_PRODUCT     "USB IO Board"
#endif

const uint8_t STRING_PRODUCT[] = USB_PRODUCT;

#if USB_VID == 0x2341
#  if defined(USB_MANUFACTURER)
#    undef USB_MANUFACTURER
#  endif
#  define USB_MANUFACTURER "Arduino LLC"
#elif !defined(USB_MANUFACTURER)
// Fall through to unknown if no manufacturer name was provided in a macro
#  define USB_MANUFACTURER "Unknown"
#endif

const uint8_t STRING_MANUFACTURER[12] = USB_MANUFACTURER;


//	DEVICE DESCRIPTOR
#if (defined CDC_ENABLED) && defined(HID_ENABLED)
const DeviceDescriptor USB_DeviceDescriptor =
	D_DEVICE(0xEF,0x02,0x01,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);
#elif defined(CDC_ENABLED)  // CDC only
const DeviceDescriptor USB_DeviceDescriptor =
	D_DEVICE(0x02,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);
#else // HID only
const DeviceDescriptor USB_DeviceDescriptor =
	D_DEVICE(0,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);
#endif

//==================================================================

volatile uint32_t _usbConfiguration = 0;
volatile uint32_t _usbInitialized = 0;
volatile uint32_t _usbSetInterface = 0;

//==================================================================


//	Number of bytes, assumes a rx endpoint
uint32_t USBD_Available(uint32_t ep)
{
	return UDD_FifoByteCount(ep);
}

//	Non Blocking receive
//	Return number of bytes read
uint32_t USBD_Recv(uint32_t ep, void* d, uint32_t len)
{
	if (!_usbConfiguration)
		return -1;

	uint8_t *buffer;
	uint8_t *data = (uint8_t *)d;

	len = min(UDD_FifoByteCount(ep), len);

	UDD_Recv_data(ep, len);
	UDD_Recv(ep, &buffer);
	for (uint32_t i=0; i<len; i++) {
		data[i] = buffer[i];
	}

	if (len && !UDD_FifoByteCount(ep)) // release empty buffer
		UDD_ReleaseRX(ep);

	return len;
}

//	Recv 1 byte if ready
uint32_t USBD_Recv(uint32_t ep)
{
	uint8_t c;
	if (USBD_Recv(ep, &c, 1) != 1)
		return -1;
	else
		return c;
}

//	Blocking Send of data to an endpoint
uint32_t USBD_Send(uint32_t ep, const void* d, uint32_t len)
{
	int r = len;
	const uint8_t* data = (const uint8_t*)d;

    if (!_usbConfiguration)
    {
    	TRACE_CORE(printf("pb conf\n\r");)
		return -1;
    }
	UDD_Send(ep, data, len);

	/* Clear the transfer complete flag  */
	udd_clear_IN_transf_cplt(ep);
	/* Set the bank as ready */
	udd_IN_transfer_allowed(ep);

	/* Wait for transfer to complete */
	while (! udd_is_IN_transf_cplt(ep));  // need fire exit.
	return r;
}

uint32_t USBD_SendControl(uint8_t flags, const void* d, uint32_t len)
{
	const uint8_t* data = (const uint8_t*)d;
	uint32_t length = len;
	uint32_t sent = 0;
	uint32_t pos = 0;

	TRACE_CORE(printf("=> USBD_SendControl TOTAL len=%lu\r\n", len);)

 		while (len > 0)
 		{
			sent = UDD_Send(EP0, data + pos, len);
			TRACE_CORE(printf("=> USBD_SendControl sent=%lu\r\n", sent);)
			pos += sent;
			len -= sent;
 		}

	return length;
}

// Send a USB descriptor string. The string is stored as a
// plain ASCII string but is sent out as UTF-16 with the
// correct 2-byte prefix
static bool USB_SendStringDescriptor(const uint8_t *string, int wLength)
{
	uint16_t buff[64];
	int l = 1;

	wLength -= 2;
	while (*string && wLength>0)
	{
		buff[l++] = (uint8_t)(*string++);
		wLength -= 2;
	}
	buff[0] = (3<<8) | (l*2);

	return USBD_SendControl(0, (uint8_t*)buff, l*2);
}

uint32_t USBD_RecvControl(void* d, uint32_t len)
{
	uint8_t *buffer;
	uint8_t *data = (uint8_t *)d;
	uint32_t read = UDD_Recv_data(EP0, len);
	if (read > len)
		read = len;
	UDD_Recv(EP0, &buffer);
	while (!udd_is_OUT_transf_cplt(EP0));
	for (uint32_t i=0; i<read; i++) {
		data[i] = buffer[i];
	}
	udd_OUT_transfer_allowed(EP0);
	return read;
}

//	Handle CLASS_INTERFACE requests
bool USBD_ClassInterfaceRequest(Setup& setup)
{
	uint8_t i = setup.wIndex;

	TRACE_CORE(printf("=> USBD_ClassInterfaceRequest\r\n");)

#ifdef CDC_ENABLED
	if (CDC_ACM_INTERFACE == i)
	{
		if( CDC_Setup(setup) == false )
		{
			send_zlp();
		}
		return true;
	}
#endif

#ifdef HID_ENABLED
	if (HID_INTERFACE == i)
	{
		if( HID_Setup(setup) == true )
		{
			send_zlp();
		}
		return true;
	}
#endif

	return false;
}

//	Construct a dynamic configuration descriptor
//	This really needs dynamic endpoint allocation etc
//	TODO
static bool USBD_SendConfiguration(uint32_t maxlen)
{
	uint8_t cache_buffer[128];
	uint8_t i;

	const uint8_t* interfaces;
	uint32_t interfaces_length = 0;
	uint8_t num_interfaces[1];

	num_interfaces[0] = 0;

#if (defined CDC_ENABLED) && defined(HID_ENABLED)
    num_interfaces[0] += 3;
    interfaces = (const uint8_t*) CDC_GetInterface();
    interfaces_length = CDC_GetInterfaceLength() + HID_GetInterfaceLength();
    if( maxlen > CDC_GetInterfaceLength() + HID_GetInterfaceLength() + sizeof(ConfigDescriptor) )
    {
	    maxlen = CDC_GetInterfaceLength() + HID_GetInterfaceLength() + sizeof(ConfigDescriptor);
    }

#else
#ifdef CDC_ENABLED
    num_interfaces[0] += 2;
	interfaces = (const uint8_t*) CDC_GetInterface();
	interfaces_length += CDC_GetInterfaceLength();
	if( maxlen > CDC_GetInterfaceLength()+ sizeof(ConfigDescriptor) )
	{
		maxlen = CDC_GetInterfaceLength()+ sizeof(ConfigDescriptor);
	}
#endif

#ifdef HID_ENABLED
    num_interfaces[0] += 1;
	interfaces = (const uint8_t*) HID_GetInterface();
	interfaces_length += HID_GetInterfaceLength();
	if( maxlen > HID_GetInterfaceLength()+ sizeof(ConfigDescriptor) )
	{
		maxlen = HID_GetInterfaceLength()+ sizeof(ConfigDescriptor);
	}
#endif
#endif

_Pragma("pack(1)")
	ConfigDescriptor config = D_CONFIG((uint16_t)(interfaces_length + sizeof(ConfigDescriptor)),num_interfaces[0]);
_Pragma("pack()")

	memcpy( cache_buffer, &config, sizeof(ConfigDescriptor) );

#if (defined CDC_ENABLED) && defined(HID_ENABLED)
	for ( i=0; i<CDC_GetInterfaceLength(); i++)
	{
		cache_buffer[i+sizeof(ConfigDescriptor)] = interfaces[i];
	}
	interfaces = (const uint8_t*) HID_GetInterface();
	for ( i=0; i<HID_GetInterfaceLength(); i++)
	{
		cache_buffer[i+sizeof(ConfigDescriptor)+CDC_GetInterfaceLength()] = interfaces[i];
	}
#else
#ifdef HID_ENABLED
	for ( i=0; i<interfaces_length; i++)
	{
		cache_buffer[i+sizeof(ConfigDescriptor)] = interfaces[i];
	}
#endif

#ifdef CDC_ENABLED
	for ( i=0; i<interfaces_length; i++)
	{
		cache_buffer[i+sizeof(ConfigDescriptor)] = interfaces[i];
	}
#endif
#endif

	if (maxlen > sizeof(cache_buffer))
	{
		 maxlen = sizeof(cache_buffer);
	}
	USBD_SendControl(0,cache_buffer, maxlen );
	return true;
}

static bool USBD_SendDescriptor(Setup* pSetup)
{
	uint8_t t = pSetup->wValueH;
	uint8_t desc_length = 0;
	const uint8_t* desc_addr = 0;

	if (USB_CONFIGURATION_DESCRIPTOR_TYPE == t)
	{
		TRACE_CORE(printf("=> USBD_SendDescriptor : USB_CONFIGURATION_DESCRIPTOR_TYPE length=%d\r\n", setup.wLength);)
		return USBD_SendConfiguration(pSetup->wLength);
	}

#ifdef HID_ENABLED
	if (HID_REPORT_DESCRIPTOR_TYPE == t)
	{
		TRACE_CORE(puts("=> USBD_SendDescriptor : HID_REPORT_DESCRIPTOR_TYPE\r\n");)
		return HID_GetDescriptor();
	}
	if (HID_HID_DESCRIPTOR_TYPE == t)
	{
		uint8_t tab[9] = D_HIDREPORT((uint8_t)HID_SizeReportDescriptor());

		TRACE_CORE(puts("=> USBD_SendDescriptor : HID_HID_DESCRIPTOR_TYPE\r\n");)

		return USBD_SendControl(0, tab, sizeof(tab));
	}
#endif

	if (USB_DEVICE_DESCRIPTOR_TYPE == t)
	{
		TRACE_CORE(puts("=> USBD_SendDescriptor : USB_DEVICE_DESCRIPTOR_TYPE\r\n");)
		desc_addr = (const uint8_t*)&USB_DeviceDescriptor;
        if( *desc_addr > pSetup->wLength ) {
            desc_length = pSetup->wLength;
        }
	}
	else if (USB_STRING_DESCRIPTOR_TYPE == t)
	{
		TRACE_CORE(puts("=> USBD_SendDescriptor : USB_STRING_DESCRIPTOR_TYPE\r\n");)
		if (pSetup->wValueL == 0) {
			desc_addr = (const uint8_t*)&STRING_LANGUAGE;
		}
		else if (pSetup->wValueL == IPRODUCT) {
			return USB_SendStringDescriptor(STRING_PRODUCT, pSetup->wLength);
		}
		else if (pSetup->wValueL == IMANUFACTURER) {
			return USB_SendStringDescriptor(STRING_MANUFACTURER, pSetup->wLength);
		}
		else {
			return false;
		}
		if( *desc_addr > pSetup->wLength ) {
			desc_length = pSetup->wLength;
		}
	}
    else
    {
        TRACE_CORE(printf("Device ERROR");)
    }

	if (desc_addr == 0)
	{
		return false;
	}

	if (desc_length == 0)
	{
		desc_length = *desc_addr;
	}

	TRACE_CORE(printf("=> USBD_SendDescriptor : desc_addr=%p desc_length=%d\r\n", desc_addr, desc_length);)
	USBD_SendControl(0, desc_addr, desc_length);

	return true;
}


void EndpointHandler(uint8_t bEndpoint)
{
#ifdef CDC_ENABLED
	if( bEndpoint == CDC_ENDPOINT_OUT )
	{
		udd_OUT_transfer_allowed(CDC_ENDPOINT_OUT);

		// Handle received bytes
		if (USBD_Available(CDC_ENDPOINT_OUT))
		{
			SerialUSB.accept();
		}
	}
	if( bEndpoint == CDC_ENDPOINT_IN )
	{
		udd_IN_stop_transfer(CDC_ENDPOINT_IN);
		/* Clear the transfer complete flag  */
		udd_clear_IN_transf_cplt(CDC_ENDPOINT_IN);

	}
	if( bEndpoint == CDC_ENDPOINT_ACM )
	{
		udd_IN_stop_transfer(CDC_ENDPOINT_ACM);
		/* Clear the transfer complete flag  */
		udd_clear_IN_transf_cplt(CDC_ENDPOINT_ACM);
	}
#endif

#ifdef HID_ENABLED
	/* Nothing to do in our example */
#endif
}


void USB_ISR(void)
{
	uint16_t flags;
	uint8_t i;
	uint8_t ept_int;

	ept_int = udd_endpoint_interrupt();

	/* Not endpoint interrupt */
	if (0 == ept_int)
	{
		udd_clear_wakeup_interrupt();
		udd_clear_eorsm_interrupt();
		udd_clear_suspend_interrupt();

		// End of bus reset
		if (Is_udd_reset())
		{
			TRACE_CORE(printf(">>> End of Reset\r\n");)
			// Reset USB address to 0
			udd_configure_address(0);

			// Configure EP 0
			UDD_InitEP(0, USB_ENDPOINT_TYPE_CONTROL);
			udd_enable_setup_received_interrupt(0);
			_usbConfiguration = 0;
			udd_ack_reset();
		}

		if (Is_udd_sof())
		{
			udd_ack_sof();
		}

	}
	else
	{
		// Endpoint interrupt
		flags = udd_read_endpoint_flag(0);

		// endpoint received setup interrupt
		if (flags & USB_DEVICE_EPINTFLAG_RXSTP)
		{
			Setup *pSetupData;

			/* Clear the Received Setup flag */
			udd_read_endpoint_flag(0) = USB_DEVICE_EPINTFLAG_RXSTP;

			UDD_Recv(EP0, (uint8_t**)&pSetupData);

			/* Clear the Bank 0 ready flag on Control OUT */
			udd_OUT_transfer_allowed(0);

			bool ok = true;
			if (REQUEST_STANDARD == (pSetupData->bmRequestType & REQUEST_TYPE))
			{
				unsigned char data_to_be_send[2];

				// Standard Requests
				uint8_t r = pSetupData->bRequest;
				if (GET_STATUS == r)
				{
					if( pSetupData->bmRequestType == 0 )  // device
					{
						// Send the device status
     					TRACE_CORE(puts(">>> EP0 Int: GET_STATUS\r\n");)
						// Check current configuration for power mode (if device is configured)
						// TODO
						// Check if remote wake-up is enabled
						// TODO
						data_to_be_send[0]=0;
						data_to_be_send[1]=0;
						UDD_Send(0, data_to_be_send, 2);
					}
					// if( pSetupData->bmRequestType == 2 ) // Endpoint:
					else
					{
						// Send the endpoint status
						// Check if the endpoint if currently halted
 						if( isEndpointHalt == 1 )
							data_to_be_send[0]=1;
 						else
							data_to_be_send[0]=0;
						data_to_be_send[1]=0;
						UDD_Send(0, data_to_be_send, 2);
					}
				}
				else if (CLEAR_FEATURE == r)
				{
				   // Check which is the selected feature
					if( pSetupData->wValueL == 1) // DEVICEREMOTEWAKEUP
					{
						// Enable remote wake-up and send a ZLP
						if( isRemoteWakeUpEnabled == 1 )
							data_to_be_send[0]=1;
						else
							data_to_be_send[0]=0;
						data_to_be_send[1]=0;
						UDD_Send(0, data_to_be_send, 2);
					}
					else // if( pSetupData->wValueL == 0) // ENDPOINTHALT
					{
						isEndpointHalt = 0;
						send_zlp();
					}
 				}
				else if (SET_FEATURE == r)
				{
					// Check which is the selected feature
					if( pSetupData->wValueL == 1) // DEVICEREMOTEWAKEUP
					{
						// Enable remote wake-up and send a ZLP
						isRemoteWakeUpEnabled = 1;
	    				data_to_be_send[0] = 0;
						UDD_Send(0, data_to_be_send, 1);
					}
					if( pSetupData->wValueL == 0) // ENDPOINTHALT
					{
						// Halt endpoint
						isEndpointHalt = 1;
						send_zlp();
					}
				}
				else if (SET_ADDRESS == r)
				{
					TRACE_CORE(puts(">>> EP0 Int: SET_ADDRESS\r\n");)
					UDD_SetAddress(pSetupData->wValueL);
				}
				else if (GET_DESCRIPTOR == r)
				{
					TRACE_CORE(puts(">>> EP0 Int: GET_DESCRIPTOR\r\n");)
					ok = USBD_SendDescriptor(pSetupData);
				}
				else if (SET_DESCRIPTOR == r)
				{
					TRACE_CORE(puts(">>> EP0 Int: SET_DESCRIPTOR\r\n");)
					ok = false;
				}
				else if (GET_CONFIGURATION == r)
				{
					TRACE_CORE(puts(">>> EP0 Int: GET_CONFIGURATION\r\n");)
					UDD_Send(0, (void*)&_usbConfiguration, 1);
				}
				else if (SET_CONFIGURATION == r)
				{
					if (REQUEST_DEVICE == (pSetupData->bmRequestType & REQUEST_RECIPIENT))
					{
						TRACE_CORE(printf(">>> EP0 Int: SET_CONFIGURATION REQUEST_DEVICE %d\r\n", pSetupData->wValueL);)
#ifdef HID_ENABLED
						UDD_InitEP( HID_ENDPOINT_INT, USB_ENDPOINT_TYPE_INTERRUPT | USB_ENDPOINT_IN(0));
#endif

#ifdef CDC_ENABLED
						UDD_InitEP( CDC_ENDPOINT_ACM, USB_ENDPOINT_TYPE_BULK | USB_ENDPOINT_IN(0));
						UDD_InitEP( CDC_ENDPOINT_OUT, USB_ENDPOINT_TYPE_BULK | USB_ENDPOINT_OUT(0));
						UDD_InitEP( CDC_ENDPOINT_IN, USB_ENDPOINT_TYPE_INTERRUPT | USB_ENDPOINT_IN(0));
#endif
						_usbConfiguration = pSetupData->wValueL;

#ifdef CDC_ENABLED
						// Enable interrupt for CDC reception from host (OUT packet)
						udd_ept_enable_it_IN_transf_cplt(CDC_ENDPOINT_ACM);
						udd_ept_enable_it_OUT_transf_cplt(CDC_ENDPOINT_OUT);
#endif
						send_zlp();
					}
					else
					{
						TRACE_CORE(puts(">>> EP0 Int: SET_CONFIGURATION failed!\r\n");)
						ok = false;
					}
				}
				else if (GET_INTERFACE == r)
				{
					TRACE_CORE(puts(">>> EP0 Int: GET_INTERFACE\r\n");)
					UDD_Send(0, (void*)&_usbSetInterface, 1);
				}
				else if (SET_INTERFACE == r)
				{
					_usbSetInterface = pSetupData->wValueL;
					TRACE_CORE(puts(">>> EP0 Int: SET_INTERFACE\r\n");)
					send_zlp();
				}
			}
			else
			{
				TRACE_CORE(puts(">>> EP0 Int: ClassInterfaceRequest\r\n");)
				ok =  USBD_ClassInterfaceRequest(*pSetupData);
			}

			if (ok)
			{
				TRACE_CORE(puts(">>> EP0 Int: Send packet\r\n");)
				UDD_ClearIN();
			}
			else
			{
				TRACE_CORE(puts(">>> EP0 Int: Stall\r\n");)
				UDD_Stall(0);
			}

			if( flags & USB_DEVICE_EPINTFLAG_STALL(2) )
			{
				/* Clear the stall flag */
				udd_clear_stall_request(0);

				// Remove stall request
				udd_remove_stall_request(0);
			}
		}  // end if USB_DEVICE_EPINTFLAG_RXSTP

		i=0;
		ept_int &= 0xFE;  // Remove endpoint number 0 (setup)
		while (ept_int != 0)
		{
            // Check if endpoint has a pending interrupt
            if ((ept_int & (1 << i)) != 0)
			{
				if( (udd_read_endpoint_flag(i) & USB_DEVICE_EPINTFLAG_TRCPT_Msk ) != 0 )

				{
					EndpointHandler(i);
				}
                ept_int &= ~(1 << i);

                if (ept_int != 0)
				{

                    TRACE_CORE("\n\r  - ");
                }
            }
            i++;
			if( i> USB_EPT_NUM) break;  // fire exit
        }
	}
}



void USBD_Flush(uint32_t ep)
{
	if (UDD_FifoByteCount(ep))
	{
		UDD_ReleaseTX(ep);
	}
}

//	Counting frames
uint32_t USBD_Connected(void)
{
	uint8_t f = UDD_GetFrameNumber();

    //delay(3);

	return f != UDD_GetFrameNumber();
}


//=======================================================================
//=======================================================================

USBDevice_ USBDevice;

USBDevice_::USBDevice_()
{
	UDD_SetStack(&USB_ISR);
}

bool USBDevice_::attach()
{
  if (_usbInitialized != 0UL)
  {
    UDD_Attach();
	_usbConfiguration = 0;
	return true;
  }
  else
  {
    return false;
  }
}

bool USBDevice_::detach()
{
	if (_usbInitialized != 0UL)
	{
		UDD_Detach();
		return true;
	}
	else
	{
		return false;
	}
}

bool USBDevice_::configured()
{
	return _usbConfiguration;
}

void USBDevice_::poll()
{
}

void USBDevice_::init()
{
	UDD_Init();
	_usbInitialized=1UL;
}
