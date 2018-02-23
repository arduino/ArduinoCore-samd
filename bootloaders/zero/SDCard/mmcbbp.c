/*-----------------------------------------------------------------------/
/  PFF - Generic low level disk control module            (C)ChaN, 2014
/------------------------------------------------------------------------/
/
/  Copyright (C) 2014, ChaN, all right reserved.
/  Copyright (c) 2017 MattairTech LLC. All right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/-----------------------------------------------------------------------*/

#include <sam.h>
#include "pff.h"
#include "../board_definitions.h"
#include "diskio.h"
#include "../util.h"


/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

static
BYTE CardType;

/* Set sd card baud rate */
void setBaud (uint32_t baud)
{
  SDCARD_SPI_MODULE->SPI.CTRLA.bit.ENABLE = 0;
  while(SDCARD_SPI_MODULE->SPI.SYNCBUSY.bit.ENABLE);

  //Synchronous arithmetic
  #if (SAMD51 && (VARIANT_MCK == 120000000ul))
    SDCARD_SPI_MODULE->SPI.BAUD.reg = ((96000000ul / (2 * baud)) - 1); // Use 96MHz SERCOM clock (100MHz maximum) when cpu running at 120MHz
  #else
    SDCARD_SPI_MODULE->SPI.BAUD.reg = ((VARIANT_MCK / (2 * baud)) - 1);
  #endif

  SDCARD_SPI_MODULE->SPI.CTRLA.bit.ENABLE = 1;
  while(SDCARD_SPI_MODULE->SPI.SYNCBUSY.bit.ENABLE);
}

/* Initialize SPI port */
inline void init_spi (void)
{
  // Attach pins to peripheral through peripheral multiplexer
  pinMux(SDCARD_SPI_PAD0);
  pinMux(SDCARD_SPI_PAD1);
  pinMux(SDCARD_SPI_PAD2);
  pinMux(SDCARD_SPI_PAD3);

  // Configure chip select pin
  pinConfig(SDCARD_SPI_CS_PORT, SDCARD_SPI_CS_PIN, OUTPUT_HIGH);

  // 
#if (SAMD21 || SAMD11)
  PM->APBCMASK.reg |= SDCARD_SPI_BUS_CLOCK_INDEX;
#elif (SAML21)
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM0 | MCLK_APBCMASK_SERCOM1 | MCLK_APBCMASK_SERCOM2 | MCLK_APBCMASK_SERCOM3 | MCLK_APBCMASK_SERCOM4 ;
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5;  // On the SAML, SERCOM5 is on the low power bridge
#elif (SAMC21)
  #if (SAMC21E)
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM0 | MCLK_APBCMASK_SERCOM1 | MCLK_APBCMASK_SERCOM2 | MCLK_APBCMASK_SERCOM3 ;
  #elif (SAMC21G) || (SAMC21J)
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM0 | MCLK_APBCMASK_SERCOM1 | MCLK_APBCMASK_SERCOM2 | MCLK_APBCMASK_SERCOM3 | MCLK_APBCMASK_SERCOM4 | MCLK_APBCMASK_SERCOM5 ;
  #endif
#elif (SAMD51)
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 ;
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 ;
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 ;
  #if (SAMD51N) || (SAMD51P)
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7 ;
  #endif
#else
  #error "mmcbbp.c: Unsupported microcontroller"
#endif

  // Reset
#if defined(PARANOIA)
  SDCARD_SPI_MODULE->SPI.CTRLA.bit.SWRST = 1;
  while(SDCARD_SPI_MODULE->SPI.CTRLA.bit.SWRST || SDCARD_SPI_MODULE->SPI.SYNCBUSY.bit.SWRST);
#endif

  // Setting clock
#if (SAMD21 || SAMD11)
  GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID( SDCARD_SPI_PER_CLOCK_INDEX ) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN );
  waitForSync();
#elif (SAML21 || SAMC21 || SAMD51)
  #if (SAMD51 && (VARIANT_MCK == 120000000ul))
    GCLK->PCHCTRL[SDCARD_SPI_PER_CLOCK_INDEX].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK6 ); // Use 96MHz SERCOM clock (100MHz maximum) when cpu running at 120MHz
  #else
    GCLK->PCHCTRL[SDCARD_SPI_PER_CLOCK_INDEX].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
  #endif
  while ( (GCLK->PCHCTRL[SDCARD_SPI_PER_CLOCK_INDEX].reg & GCLK_PCHCTRL_CHEN) == 0 );      // wait for sync
#else
  #error "mmcbbp.c: Unsupported microcontroller"
#endif

  SDCARD_SPI_MODULE->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(SERCOM_MODE_SPI_MASTER_Val) |
                          ( 0 << SERCOM_SPI_CTRLA_CPHA_Pos ) |
                          ( 0 << SERCOM_SPI_CTRLA_CPOL_Pos ) |
                          SDCARD_SPI_PAD_SETTINGS |
                          SERCOM_DORD_MSB_FIRST_Val << SERCOM_SPI_CTRLA_DORD_Pos;

  SDCARD_SPI_MODULE->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(SERCOM_CHAR_SIZE_8_BITS_Val) |
                          SERCOM_SPI_CTRLB_RXEN;        //Active the SPI receiver.

  setBaud (SDCARD_SPI_BUAD_SLOW);       // also enables SPI peripheral
}

/* Select MMC */
void deselect (void)
{
  pinConfig(SDCARD_SPI_CS_PORT, SDCARD_SPI_CS_PIN, OUTPUT_HIGH);
}

/* Deselect MMC */
inline void select (void)
{
  pinConfig(SDCARD_SPI_CS_PORT, SDCARD_SPI_CS_PIN, OUTPUT_LOW);
}

/* Send a byte to the MMC or send a 0xFF to the MMC and get the received byte */
BYTE xfer_spi (BYTE data)
{
  SDCARD_SPI_MODULE->SPI.DATA.bit.DATA = data; // Writing data into Data register

  while( SDCARD_SPI_MODULE->SPI.INTFLAG.bit.RXC == 0 )
  {
    // Waiting Complete Reception
  }

  return SDCARD_SPI_MODULE->SPI.DATA.bit.DATA;  // Reading data
}


/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (
	BYTE cmd,		/* 1st byte (Start + Index) */
	DWORD arg		/* Argument (32 bits) */
)
{
	BYTE n, res;


	if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card */
	select();

	/* Send a command packet */
	xfer_spi(cmd);						/* Start + Command index */
	xfer_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xfer_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xfer_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xfer_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xfer_spi(n);

	/* Receive a command response */
	n = 10;								/* Wait for a valid response in timeout of 10 attempts */
	do {
		res = xfer_spi(DUMMY_BYTE);
	} while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}




/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (void)
{
	BYTE n, cmd, ty, ocr[4];
	UINT tmr;

        deselect();
	init_spi();							/* Initialize ports to control MMC */
	for (n = 100; n; n--) delayUs(1000);	/* 1ms delay */
	for (n = 10; n; n--) xfer_spi(DUMMY_BYTE);	/* 80 Dummy clocks with CS=H */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2 */
			for (n = 0; n < 4; n++) ocr[n] = xfer_spi(DUMMY_BYTE);		/* Get trailing return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {			/* The card can work at vdd range of 2.7-3.6V */
				for (tmr = 10000; tmr && send_cmd(ACMD41, 1UL << 30); tmr--) delayUs(100);	/* Wait for leaving idle state (ACMD41 with HCS bit) */
				if (tmr && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = xfer_spi(DUMMY_BYTE);
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 (HC or SC) */
				}
			}
		} else {							/* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
			}
			for (tmr = 10000; tmr && send_cmd(cmd, 0); tmr--) delayUs(100);	/* Wait for leaving idle state */
			if (!tmr || send_cmd(CMD16, 512) != 0)			/* Set R/W block length to 512 */
				ty = 0;
		}
	}
	CardType = ty;
        deselect();

        if (ty) {                    /* OK */
          setBaud (SDCARD_SPI_BUAD_FAST);   /* Set fast clock */
        }

	return ty ? 0 : STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read partial sector                                                   */
/*-----------------------------------------------------------------------*/

DRESULT disk_readp (
	BYTE *buff,		/* Pointer to the read buffer (NULL:Read bytes are forwarded to the stream) */
	DWORD sector,	/* Sector number (LBA) */
	UINT offset,	/* Byte offset to read from (0..511) */
	UINT count		/* Number of bytes to read (ofs + cnt mus be <= 512) */
)
{
	DRESULT res;
	BYTE rc;
	UINT bc;


	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	res = RES_ERROR;
	if (send_cmd(CMD17, sector) == 0) {		/* READ_SINGLE_BLOCK */

		bc = 40000;
		do {							/* Wait for data packet */
			rc = xfer_spi(DUMMY_BYTE);
		} while (rc == 0xFF && --bc);

		if (rc == 0xFE) {				/* A data packet arrived */
			bc = 514 - offset - count;

			/* Skip leading bytes */
			if (offset) {
				do xfer_spi(DUMMY_BYTE); while (--offset);
			}

			/* Receive a part of the sector */
			do {
				*buff++ = xfer_spi(DUMMY_BYTE);
			} while (--count);

			/* Skip trailing bytes and CRC */
			do xfer_spi(DUMMY_BYTE); while (--bc);

			res = RES_OK;
		}
	}

	deselect();

	return res;
}


