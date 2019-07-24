#ifndef __CDC_H__
#define __CDC_H__

#ifdef CDC_ENABLED

#include "USBDesc.h"
#include "USBAPI.h"
#include "PluggableUSB.h"


#define CDC_V1_10                               0x0110
#define CDC_COMMUNICATION_INTERFACE_CLASS       0x02

#define CDC_CALL_MANAGEMENT                     0x01
#define CDC_ABSTRACT_CONTROL_MODEL              0x02
#define CDC_HEADER                              0x00
#define CDC_ABSTRACT_CONTROL_MANAGEMENT         0x02
#define CDC_UNION                               0x06
#define CDC_CS_INTERFACE                        0x24
#define CDC_CS_ENDPOINT                         0x25
#define CDC_DATA_INTERFACE_CLASS                0x0A

//	CDC CS interface descriptor
typedef struct
{
	uint8_t len;		// 5
	uint8_t dtype;		// 0x24
	uint8_t subtype;
	uint8_t d0;
	uint8_t d1;
} CDCCSInterfaceDescriptor;

typedef struct
{
	uint8_t len;		// 4
	uint8_t dtype;		// 0x24
	uint8_t subtype;
	uint8_t d0;
} CDCCSInterfaceDescriptor4;

typedef struct
{
    uint8_t	len;
    uint8_t 	dtype;		// 0x24
    uint8_t 	subtype;	// 1
    uint8_t 	bmCapabilities;
    uint8_t 	bDataInterface;
} CMFunctionalDescriptor;

typedef struct
{
    uint8_t	len;
    uint8_t 	dtype;		// 0x24
    uint8_t 	subtype;	// 1
    uint8_t 	bmCapabilities;
} ACMFunctionalDescriptor;

typedef struct
{
	//	IAD
	IADDescriptor				iad;	// Only needed on compound device
	//	Control
	InterfaceDescriptor			cif;
	CDCCSInterfaceDescriptor	header;
	ACMFunctionalDescriptor		controlManagement;		// ACM
	CDCCSInterfaceDescriptor	functionalDescriptor;	// CDC_UNION
	CMFunctionalDescriptor		callManagement;			// Call Management
	EndpointDescriptor			cifin;

	//	Data
	InterfaceDescriptor			dif;
	EndpointDescriptor			in;
	EndpointDescriptor			out;
} CDCDescriptor;


//================================================================================
//	Serial over CDC (Serial1 is the physical port)

class Serial_ : public Stream, public PluggableUSBModule {
public:
	Serial_(USBDeviceClass &_usb);

	void begin(unsigned long baud_count);
	void begin(unsigned long, uint16_t);
	void end(void);

	virtual int available(void);
	virtual int availableForWrite(void);
	virtual int peek(void);
	virtual int read(void);
	virtual void flush(void);
	virtual void clear(void);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buffer, size_t size);
	using Print::write; // pull in write(str) from Print
	operator bool();

	size_t readBytes(char *buffer, size_t length);

	// This method allows processing "SEND_BREAK" requests sent by
	// the USB host. Those requests indicate that the host wants to
	// send a BREAK signal and are accompanied by a single uint16_t
	// value, specifying the duration of the break. The value 0
	// means to end any current break, while the value 0xffff means
	// to start an indefinite break.
	// readBreak() will return the value of the most recent break
	// request, but will return it at most once, returning -1 when
	// readBreak() is called again (until another break request is
	// received, which is again returned once).
	// This also mean that if two break requests are received
	// without readBreak() being called in between, the value of the
	// first request is lost.
	// Note that the value returned is a long, so it can return
	// 0-0xffff as well as -1.
	int32_t readBreak();

	// These return the settings specified by the USB host for the
	// serial port. These aren't really used, but are offered here
	// in case a sketch wants to act on these settings.
	unsigned long baud();
	uint8_t stopbits();
	uint8_t paritytype();
	uint8_t numbits();
	bool dtr();
	bool rts();
	enum {
		ONE_STOP_BIT = 0,
		ONE_AND_HALF_STOP_BIT = 1,
		TWO_STOP_BITS = 2,
	};
	enum {
		NO_PARITY = 0,
		ODD_PARITY = 1,
		EVEN_PARITY = 2,
		MARK_PARITY = 3,
		SPACE_PARITY = 4,
	};

protected:
    // Implementation of the PUSBListNode
    int getInterface(uint8_t* interfaceNum);
    int getDescriptor(USBSetup& setup);
    bool setup(USBSetup& setup);
    uint8_t getShortName(char* name);
    void handleEndpoint(int ep);
    void enableInterrupt();

friend USBDeviceClass;

private:
	int availableForStore(void);

	USBDeviceClass &usb;
	bool stalled;
	uint32_t epType[3];
};
extern Serial_ SerialUSB;

#endif
#endif
