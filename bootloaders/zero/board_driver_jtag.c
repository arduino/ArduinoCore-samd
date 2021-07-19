#include "board_driver_jtag.h"
#include <string.h>

#ifdef ENABLE_JTAG_LOAD

/* JTAG State Machine */
const int JSM[16][2] = {
  /*-State-      -mode= '0'-    -mode= '1'- */
  /*RESET     */ {JS_RUNIDLE,   JS_RESET    },
  /*RUNIDLE   */ {JS_RUNIDLE,   JS_SELECT_DR},
  /*SELECTIR  */ {JS_CAPTURE_IR, JS_RESET    },
  /*CAPTURE_IR*/ {JS_SHIFT_IR,  JS_EXIT1_IR },
  /*SHIFT_IR  */ {JS_SHIFT_IR,  JS_EXIT1_IR },
  /*EXIT1_IR  */ {JS_PAUSE_IR,  JS_UPDATE_IR},
  /*PAUSE_IR  */ {JS_PAUSE_IR,  JS_EXIT2_IR },
  /*EXIT2_IR  */ {JS_SHIFT_IR,  JS_UPDATE_IR},
  /*UPDATE_IR */ {JS_RUNIDLE,   JS_SELECT_DR},
  /*SELECT_DR */ {JS_CAPTURE_DR, JS_SELECT_IR},
  /*CAPTURE_DR*/ {JS_SHIFT_DR,  JS_EXIT1_DR },
  /*SHIFT_DR  */ {JS_SHIFT_DR,  JS_EXIT1_DR },
  /*EXIT1_DR  */ {JS_PAUSE_DR,  JS_UPDATE_DR},
  /*PAUSE_DR  */ {JS_PAUSE_DR,  JS_EXIT2_DR },
  /*EXIT2_DR  */ {JS_SHIFT_DR,  JS_UPDATE_DR},
  /*UPDATE_DR */ {JS_RUNIDLE,   JS_SELECT_DR}
};


static struct states {
  unsigned char state;
  unsigned char nSlaves;
  unsigned char slaveBits;
  unsigned char virSize;
  unsigned char id;
  unsigned char lastVir;
} jtag;

void port_pin_set_output_level(int pin, int level) {
  if (level) {
    outpin_on(pin);
  } else {
    outpin_off(pin);
  }
}

int port_pin_get_input_level(int pin) {
  return inpin_get(pin);
}



/******************************************************************/
/* Name:         DriveSignal                                      */
/*                                                                */
/* Parameters:   signal,data,clk,buffer_enable                    */
/*               -the name of the signal (SIG_*).                 */
/*               -the value to be dumped to the signal,'1' or '0' */
/*               -driving a LOW to HIGH transition to SIG_TCK     */
/*                together with signal.                           */
/*               -buffer_enable is used by WritePort function.	  */
/*				 -If "buffer_enable"=1,							  */
/*				 -processes in "port_io_buffer" are flushed when  */
/*               -"PORT_IO_BUFFER_SIZE" is reached.				  */
/*				 -If "buffer_enable"=0,							  */
/*               -"data" is dumped to port 0 at once			  */
/*                                                                */
/* Return Value: None.                                            */
/*                                                                */
/* Descriptions: Dump data to signal. If clk is '1', a clock pulse*/
/*               is driven after the data is dumped to signal.    */
/*                                                                */
/******************************************************************/
static void DriveSignal(int signal, int data, int clk)
{

  port_pin_set_output_level (signal, data);

  if (clk)
  {
    port_pin_set_output_level (TCK, 1);
    port_pin_set_output_level (TCK, 0);
  }
}


/******************************************************************/
/* Name:         ReadTDO                                          */
/*                                                                */
/* Parameters:   bit_count,data,inst                              */
/*               -bit_count is the number of bits to shift out.   */
/*               -data is the value to shift in from lsb to msb.  */
/*               -inst determines if the data is an instruction.  */
/*                if inst=1,the number of bits shifted in/out     */
/*                equals to bit_count-1;if not,the number of bits */
/*                does not change.                                */
/*                                                                */
/* Return Value: The data shifted out from TDO. The first bit     */
/*               shifted out is placed at the lsb of the returned */
/*               integer.                                         */
/*               		                                          */
/* Descriptions: Shift out bit_count bits from TDO while shift in */
/*               data to TDI. During instruction loading, the     */
/*               number of shifting equals to the instruction     */
/*               length minus 1                                   */
/*                                                                */
/******************************************************************/

static int ReadTDO(int bit_count, int data, int inst)
{
  unsigned int record = 0;
  unsigned int i;

  for (i = 0; i < bit_count; i++)
  {
    record = record | (port_pin_get_input_level(TDO) << i);

    DriveSignal(
      TDI,
      data & 1,
      !(i == (bit_count - 1) && inst)
    );

    data >>= 1;
  }
  return record;
} /*
  int ReadTDO(int bit_count,int data,int inst)
  {
	unsigned int tdi=0,tdo=0,record=0;
	unsigned int i;

	for(i=0;i<bit_count;i++)
	{
		unsigned int mask=1;

		tdo=port_pin_get_input_level (PIN_TDO);

		tdo = tdo? (1<<i):0;
		record = record | tdo;
		mask = mask << i;
		tdi = data & mask;
		tdi = tdi >> i;
		if (i==(bit_count-1) && inst)
		  DriveSignal(PIN_TDI,tdi,0);
		else
		  DriveSignal(PIN_TDI,tdi,1);
	}
	return record;
  }
*/
/******************************************************************/
/* Name:         ReadTDO                                          */
/*                                                                */
/* Parameters:   bit_count,data,inst                              */
/*               -bit_count is the number of bits to shift out.   */
/*               -data is the value to shift in from lsb to msb.  */
/*               -inst determines if the data is an instruction.  */
/*                if inst=1,the number of bits shifted in/out     */
/*                equals to bit_count-1;if not,the number of bits */
/*                does not change.                                */
/*                                                                */
/* Return Value: The data shifted out from TDO. The first bit     */
/*               shifted out is placed at the lsb of the returned */
/*               integer.                                         */
/*               		                                          */
/* Descriptions: Shift out bit_count bits from TDO while shift in */
/*               data to TDI. During instruction loading, the     */
/*               number of shifting equals to the instruction     */
/*               length minus 1                                   */
/*                                                                */
/******************************************************************/
static void ReadTDOBuf(int bit_count, char *txbuf, char *rxbuf, int inst)
{
  unsigned int tdi = 0, tdo = 0, record = 0;
  unsigned int i;
  unsigned int charbit = 0;
  unsigned char indata, outdata;
  indata = 0;
  for (i = 0; i < bit_count; i++)
  {
    unsigned int mask = 1;

    if (charbit == 0)
      if (txbuf)
        outdata = *txbuf++;
      else
        outdata = -1;

    indata = (indata >> 1) | (port_pin_get_input_level (TDO) << 7);

    DriveSignal( TDI,
                 outdata & 1,
                 !(i == (bit_count - 1) && inst)
               );
    outdata = outdata >> 1;
    charbit = (charbit + 1) & 7;
    if (charbit == 0 && rxbuf)
    {
      *rxbuf++ = indata;
      indata = 0;
    }
  }
}

/******************************************************************/
/* Name:         AdvanceJSM                                       */
/*                                                                */
/* Parameters:   mode                                             */
/*               -the input mode to JSM.                          */
/*                                                                */
/* Return Value: The current JSM state.                           */
/*               		                                          */
/* Descriptions: Function that keep track of the JSM state. It    */
/*               drives out signals to TMS associated with a      */
/*               clock pulse at TCK and updates the current state */
/*               variable.                                        */
/*                                                                */
/******************************************************************/
static int AdvanceJSM(int mode)
{
  DriveSignal(TMS, mode, 1);

  jtag.state = JSM[jtag.state][mode];

  return (jtag.state);
}

/******************************************************************/
/* Name:         Js_Updatedr                                      */
/*                                                                */
/* Parameters:   None.                                            */
/*                                                                */
/* Return Value: 1 if the current state is not SHIFT_DR;0 if the  */
/*               operation is successful.                         */
/*               		                                          */
/* Descriptions: Move the JSM to UPDATE_DR. The current state is  */
/*               expected to be SHIFT_DR                          */
/*                                                                */
/******************************************************************/
static int Js_Updatedr()
{
  /* The current JSM state must be in UPDATE_IR or UPDATE_DR */
  if (jtag.state != JS_SHIFT_DR)
    return (1);

  AdvanceJSM(1);
  AdvanceJSM(1);

  return (0);
}

/******************************************************************/
/* Name:         Js_Shiftdr                                       */
/*                                                                */
/* Parameters:   None.                                            */
/*                                                                */
/* Return Value: 1 if the current state is not UPDATE_DR or       */
/*               UPDATE_IR. 0 if the opeation is successful.      */
/*               		                                          */
/* Descriptions: Move the JSM to SHIFT_DR. The current state is   */
/*               expected to be UPDATE_DR or UPDATE_IR.           */
/*                                                                */
/******************************************************************/
static int Js_Shiftdr()
{
  /* The current JSM state must be in UPDATE_IR or UPDATE_DR */
  if (jtag.state != JS_UPDATE_DR && jtag.state != JS_UPDATE_IR)
  {
    if (jtag.state != JS_RESET && jtag.state != JS_RUNIDLE)
      return (1);
    else
    {
      AdvanceJSM(0);
      AdvanceJSM(0);
      AdvanceJSM(1);
      AdvanceJSM(0);
      AdvanceJSM(0);

      return (0);
    }
  }

  AdvanceJSM(1);
  AdvanceJSM(0);
  AdvanceJSM(0);

  return (0);
}

/******************************************************************/
/* Name:         Js_Reset                                         */
/*                                                                */
/* Parameters:   None.                                            */
/*                                                                */
/* Return Value: None.                                            */
/*               		                                          */
/* Descriptions: Reset the JSM by issuing JSM_RESET_COUNT of clock*/
/*               with the TMS at HIGH.                            */
/*                                                                */
/******************************************************************/
static void Js_Reset()
{
  int i;

  for (i = 0; i < JSM_RESET_COUNT; i++)
    AdvanceJSM(1);
}

/******************************************************************/
/* Name:         Runidle                                          */
/*                                                                */
/* Parameters:   None.                                            */
/*                                                                */
/* Return Value: None.                                            */
/*               		                                          */
/* Descriptions: If the current JSM is not at UPDATE_DR or        */
/*               UPDATE_IR state, RESET JSM and move to RUNIDLE,  */
/*               if it is, clock once with TMS LOW and move to    */
/*               RUNIDLE.                                         */
/*                                                                */
/******************************************************************/
static void Js_Runidle()
{
  int i = 0;

  /* If the current state is not UPDATE_DR or UPDATE_IR, reset the JSM and move to RUN/IDLE */
  if (jtag.state != JS_UPDATE_IR && jtag.state != JS_UPDATE_DR)
  {
    for (i = 0; i < JSM_RESET_COUNT; i++)
      AdvanceJSM(1);
  }

  AdvanceJSM(0);
}

/******************************************************************/
/* Name:         LoadJI                                           */
/*                                                                */
/* Parameters:   action,dev_count,ji_info                         */
/*               -action is the JTAG instruction to load          */
/*               -dev_count is the maximum number of devices in   */
/*                chain.                                          */
/*               -ji_info is the pointer to an integer array that */
/*                contains the JTAG instruction length for the    */
/*                devices in chain.                               */
/*                                                                */
/* Return Value: 1 if contains error;0 if not.                    */
/*               		                                          */
/* Descriptions: Move the JSM to SHIFT_IR. Load in the JTAG       */
/*               instruction to all devices in chain. Then        */
/*               advance the JSM to UPDATE_IR. Irrespective of    */
/*                                                                */
/******************************************************************/
static int LoadJI(int action)
{
  int i, record = 0, error = 0;

  /* Move Jtag State Machine (JSM) to RUN/IDLE */
  if (jtag.state != JS_RUNIDLE && jtag.state != JS_RESET)
    Js_Runidle();

  /* Move JSM to SHIFT_IR */
  AdvanceJSM(0);
  AdvanceJSM(1);
  AdvanceJSM(1);
  AdvanceJSM(0);
  AdvanceJSM(0);

  record = ReadTDO(INST_LEN, action, 1);
  if (record != 0x155)
  {
    error = -1;
    //			fprintf(stderr,"Error: JTAG chain broken!\nError: Bits unloaded: 0x%X\n", record);
    return error;
  }

  /* Move JSM to UPDATE_IR */
  AdvanceJSM(1);
  AdvanceJSM(1);

  return error;
}

/******************************************************************/
/* Name:         SetupChain                                       */
/*                                                                */
/* Parameters:   dev_count,dev_seq,ji_info,action                 */
/*               -device_count is the total device in chain       */
/*               -dev_seq is the device sequence in chain         */
/*               -ji_info is the pointer to an integer array that */
/*                contains the JTAG instruction length for the    */
/*                devices in chain.                               */
/*               -action is the JTAG instruction to load          */
/*                                                                */
/* Return Value: None.                                            */
/*               		                                          */
/* Descriptions: Move the JSM to SHIFT_IR. Issue the JTAG         */
/*               instruction, "action" to the target device and   */
/*               BYPASS to the rest of the devices. Then, move    */
/*               the JSM to UPDATE_IR.                            */
/*                                                                */
/******************************************************************/
static void SetupChain(int action)
{
  int i, record = 0;
  /* Move Jtag State Machine (JSM) to RUN/IDLE */
  if (jtag.state != JS_RUNIDLE && jtag.state != JS_RESET)
    Js_Runidle();

  /* Move JSM to SHIFT_IR */
  AdvanceJSM(0);
  AdvanceJSM(1);
  AdvanceJSM(1);
  AdvanceJSM(0);
  AdvanceJSM(0);

  record = ReadTDO(INST_LEN, action, 1);

  /* Move JSM to UPDATE_IR */
  AdvanceJSM(1);
  AdvanceJSM(1);
}

/******************************************************************/
/* Name:         CheckStatus                                      */
/*                                                                */
/* Parameters:   dev_seq                                          */
/*               -dev_seq is the device sequence in chains.       */
/*                                                                */
/* Return Value: '0' if CONF_DONE is HIGH;'1' if it is LOW.       */
/*               		                                          */
/* Descriptions: Issue CHECK_STATUS instruction to the device to  */
/*               be configured and BYPASS for the rest of the     */
/*               devices.                                         */
/*                                                                */
/*               <conf_done_bit> =                                */
/*                  ((<Maximum JTAG sequence> -                   */
/*                    <JTAG sequence for CONF_DONE pin>)*3) + 1   */
/*                                                                */
/*               The formula calculates the number of bits        */
/*               to be shifted out from the device, excluding the */
/*               1-bit register for each device in BYPASS mode.   */
/*                                                                */
/******************************************************************/
int CheckStatus()
{
	int bit,data=0,error=0;
	int jseq_max=0,jseq_conf_done=0,conf_done_bit=0;

	//	fprintf( stdout, "Info: Checking Status\n" );

	/* Load CHECK_STATUS instruction */
	SetupChain(JI_CHECK_STATUS);

	Js_Shiftdr();

	/* Maximum JTAG sequence of the device in chain */
	jseq_max= JSEQ_MAX;

	jseq_conf_done= JSEQ_CONF_DONE;

	conf_done_bit = ((jseq_max-jseq_conf_done)*3)+1;

	/* Compensate for 1 bit unloaded from every Bypass register */
	conf_done_bit+= 0;
	
	for(bit=0;bit<conf_done_bit;bit++)
	{
		DriveSignal(TDI,0,1);
	}

	data = ReadTDO(1,0,0);

	if(!data)
	{
		error++;
	}

	/* Move JSM to RUNIDLE */
	Js_Updatedr();
	Js_Runidle();

	return (error);
}

static int jtagVIR(int instruction)
{
  int ret = 0;
  if (jtag.lastVir != instruction) {
    int code = ((jtag.id + 1) << jtag.virSize) | instruction;
    ret = LoadJI(JI_USER1_VIR); // Ji_Active_Disengage(device_count,ji_info);
    if (ret < 0) {
		return ret;
	}
    Js_Shiftdr();
    ReadTDO(jtag.virSize + jtag.slaveBits, code, 1);
    Js_Updatedr();
    jtag.lastVir = instruction;
  }
  return ret;
}

int jtagInit(void)
{
  int i, j;
  unsigned int record;

  inpin_init(TDO);
  outpin_init(TMS);
  outpin_init(TDI);
  outpin_init(TCK);

  mbPinSet();

  port_pin_set_output_level (TMS, 1);
  port_pin_set_output_level (TDI, 1);
  port_pin_set_output_level (TCK, 0);

  Js_Runidle();

  if (CheckStatus()==0)
  {
    LoadJI(JI_USER1_VIR); 

	  Js_Shiftdr();
	  ReadTDO(64, 0, 0);
	  Js_Updatedr();
	  LoadJI(JI_USER0_VDR); // Ji_Active_Disengage(device_count,ji_info);
	  record = 0;
	  for (i = 0; i < 8; i++)
	  {
		Js_Shiftdr();
		record = (record >> 4) | (ReadTDO(4, 0x0, 0) << 28);
		Js_Updatedr();
		Js_Runidle();
	  }
	  jtag.id = -1;
	  jtag.lastVir = -1;
	  if (((record >> 8) & 0x7ff) == JTAG_VENDOR_ID)
	  {
		jtag.nSlaves = (record >> 19) & 0xff; // number of jtag slaves
		for (jtag.slaveBits = 0; (1 << jtag.slaveBits) < (jtag.nSlaves + 1); jtag.slaveBits++);

		jtag.virSize = record & 0xff;
		for (j = 0; j < jtag.nSlaves; j++)
		{
		  record = 0;
		  for (i = 0; i < 8; i++)
		  {
			Js_Shiftdr();
			record = (record >> 4) | (ReadTDO(4, 0x0, 0) << 28);
			Js_Updatedr();
			Js_Runidle();
		  }
		  if (((record >> 19) & 0xff) == JTAG_ID_VJTAG && ((record >> 8) & 0x7ff) == JTAG_VENDOR_ID)
		  {
			jtag.id = j;
			return 0;
		  }
		}
	  }
  }
  return -1;
}

void jtagDeinit(void)
{
  jtag.id = -1;
}

int jtagReload() {
  int ret = LoadJI(JI_PULSE_NCONFIG);
  Js_Shiftdr();
  return ret;
}

int jtagWriteBuffer(unsigned int address, const uint8_t *data, size_t len)
{
  int ret = 0;
  ret = jtagVIR(JBC_WRITE);
  if (ret < 0) {
	return ret;
  }
  LoadJI(JI_USER0_VDR);
  Js_Shiftdr();
  address = (address << 2) | 0x00000003;
  ReadTDOBuf(32, &address, 0, 0);
  ReadTDOBuf(32 * len+2, data, 0, 0);
  return len;
}

int jtagReadBuffer(unsigned int address, uint8_t *data, size_t len)
{
  int ret = 0;
  ret = jtagVIR(JBC_WRITE);
  if (ret < 0) {
	return ret;
  }
  LoadJI(JI_USER0_VDR);
  Js_Shiftdr();
  address = (address << 2) | 0x00000003;
  ReadTDOBuf(32, &address, 0, 0);
  if (len > 1)
  {
    address = len - 1;
    ReadTDOBuf(4, &address, 0, 1);
  }
  ret = jtagVIR(JBC_READ);
  if (ret < 0) {
	return ret;
  }
  LoadJI(JI_USER0_VDR);
  Js_Shiftdr();
  for (; len > 0; len--)
  {
    //*data++=ReadTDO(32,*data,0);
    ReadTDOBuf(32, 0, data, 0);
	data += 4;
  }
  return len;
}

#define MB_BASE     0x00000000
#define MB_TIMEOUT  5000

/**
 */
int mbPinSet(void)
{
#ifdef MB_INT
  uint32_t rpc[1];
  rpc[0] = 0;
  jtagWriteBuffer(MB_BASE, (const uint8_t *)rpc, 1);
  outpin_init(MB_INT);
  outpin_off(MB_INT);
#endif
}

/**
 * Sends len words (32 bit) via messagebox
 */
int mbCmdSend(uint32_t* data, int len)
{
  int ret;
#ifdef MB_INT
  ret = jtagWriteBuffer(MB_BASE, (const uint8_t *)data, len);
  if (ret!=len) {
    return -10;
  }
  outpin_on(MB_INT);
  outpin_off(MB_INT);
#else
  jtagWriteBuffer(MB_BASE + 1, (const uint8_t *)(&data[1]), len-1);
  jtagWriteBuffer(MB_BASE, (const uint8_t *)data, 1);
#endif

  int retries = 1000;
  do {
    if (retries-- < 0) {
      return -1;
    }

    jtagReadBuffer(MB_BASE, (uint8_t*)&ret, 1);
  } while (ret);

  jtagReadBuffer(MB_BASE + 1, (uint8_t*)&ret, 1);

  return ret;
}

/**
 * Writes len words (32 bit) via messagebox at a specified address
 */
int mbWrite(uint32_t address, void* data, int len)
{
  jtagWriteBuffer(MB_BASE + address, (const uint8_t *)data, len);
  return 0;
}

/**
 * Reads len words (32 bit) using messagebox from a specified address
 */
int mbRead(uint32_t address, void* data, int len)
{
  uint32_t *p = (uint32_t*)data;
  int i;

  for (i=0; i<len; i++) {
    jtagReadBuffer(MB_BASE + address + i, (uint8_t*)&p[i], 1);
  }
  return 0;
}

#define MB_DEV_FLASH    0x01000000

uint32_t jtagBitstreamVersion()
{
  uint32_t ptr[1];
  uint32_t ver;

  ptr[0] = 0 | 1;
  ver = mbCmdSend(ptr, 1);

  return ver;
}

void jtagFlashEraseBlock(uint32_t offset)
{
  uint32_t rpc[256];
  rpc[0] = MB_DEV_FLASH | 0x03;
  rpc[1] = 2;
  rpc[2] = offset;

  mbCmdSend(rpc, 3);
}

void jtagFlashWriteBlock(uint32_t offset, size_t len, uint32_t* data)
{
  uint32_t rpc[256];
  rpc[0] = MB_DEV_FLASH | 0x04;
  rpc[1] = offset;
  rpc[2] = len;
  memcpy(&rpc[3], data, len);
  mbCmdSend(rpc, 3+((len + 3)/4));
}

void jtagFlashReadBlock(uint32_t offset, size_t len, uint8_t* buf)
{
  uint32_t rpc[256];
  rpc[0] = MB_DEV_FLASH | 0x05;
  rpc[1] = offset;
  rpc[2] = len;

  mbCmdSend(rpc, 3);
  mbRead(2, &rpc[2], (len + 3) / 4 + 1);

  uint8_t* newbuf = (uint8_t*)&rpc[3];
  for (int i = 0; i < len; i++) {
    //buf[i] = reverse(newbuf[i]);
    buf[i] = newbuf[i];
  }
}

void clockout(uint32_t gclk, int32_t divisor)
{
    GCLK_GENDIV_Type gendiv =
    {
        .bit.DIV = divisor,      // divider, linear or 2^(.DIV+1)
        .bit.ID  = gclk,         // GCLK_GENERATOR_X
    };
    GCLK->GENDIV.reg = gendiv.reg;

    // setup Clock Generator
    GCLK_GENCTRL_Type genctrl =
    {
        .bit.RUNSTDBY = 0,        // Run in Standby
        .bit.DIVSEL = 0,          // .DIV (above) Selection: 0=linear 1=powers of 2
        .bit.OE = 1,              // Output Enable to observe on a port pin
        .bit.OOV = 0,             // Output Off Value
        .bit.IDC = 1,             // Improve Duty Cycle
        .bit.GENEN = 1,           // enable this GCLK
        // select GCLK source
        //.bit.SRC = GCLK_SOURCE_OSC8M,
        .bit.SRC = GCLK_SOURCE_DFLL48M,
        // select GCLK2 to output on
        .bit.ID = gclk,           // GCLK_GENERATOR_X
    };
    GCLK->GENCTRL.reg = genctrl.reg;
}
#endif
