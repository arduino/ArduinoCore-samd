/*-----------------------------------------------------------------------
/  PFF - Low level disk interface modlue include file    (C)ChaN, 2014
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED
#define _DISKIO_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include <sam.h>
#include "integer.h"


/* Status of Disk Functions */
typedef BYTE	DSTATUS;


/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Function succeeded */
	RES_ERROR,		/* 1: Disk error */
	RES_NOTRDY,		/* 2: Not ready */
	RES_PARERR		/* 3: Invalid parameter */
} DRESULT;


/* SERCOM SPI available pad settings */
enum spi_pad_settings {
  SPI_RX_PAD0_TX_PAD2_SCK_PAD3 = SERCOM_SPI_CTRLA_DIPO(0) | SERCOM_SPI_CTRLA_DOPO(1),
  SPI_RX_PAD0_TX_PAD3_SCK_PAD1 = SERCOM_SPI_CTRLA_DIPO(0) | SERCOM_SPI_CTRLA_DOPO(2),
  SPI_RX_PAD1_TX_PAD2_SCK_PAD3 = SERCOM_SPI_CTRLA_DIPO(1) | SERCOM_SPI_CTRLA_DOPO(1),
  SPI_RX_PAD1_TX_PAD0_SCK_PAD3 = SERCOM_SPI_CTRLA_DIPO(1) | SERCOM_SPI_CTRLA_DOPO(3),
  SPI_RX_PAD2_TX_PAD0_SCK_PAD1 = SERCOM_SPI_CTRLA_DIPO(2) | SERCOM_SPI_CTRLA_DOPO(0),
  SPI_RX_PAD2_TX_PAD3_SCK_PAD1 = SERCOM_SPI_CTRLA_DIPO(2) | SERCOM_SPI_CTRLA_DOPO(2),
  SPI_RX_PAD2_TX_PAD0_SCK_PAD3 = SERCOM_SPI_CTRLA_DIPO(2) | SERCOM_SPI_CTRLA_DOPO(3),
  SPI_RX_PAD3_TX_PAD0_SCK_PAD1 = SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0),
};


/*---------------------------------------*/
/* Prototypes for disk control functions */

DSTATUS disk_initialize (void);
DRESULT disk_readp (BYTE* buff, DWORD sector, UINT offser, UINT count);
DRESULT disk_writep (const BYTE* buff, DWORD sc);

#define DUMMY_BYTE			0xFF

#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */

/* Card type flags (CardType) */
#define CT_MMC				0x01	/* MMC ver 3 */
#define CT_SD1				0x02	/* SD ver 1 */
#define CT_SD2				0x04	/* SD ver 2 */
#define CT_SDC				(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK			0x08	/* Block addressing */

/* Definitions for MMC/SDC command */
#define CMD0    (0x40+0)        /* GO_IDLE_STATE */
#define CMD1    (0x40+1)        /* SEND_OP_COND (MMC) */
#define ACMD41  (0xC0+41)       /* SEND_OP_COND (SDC) */
#define CMD8    (0x40+8)        /* SEND_IF_COND */
#define CMD16   (0x40+16)       /* SET_BLOCKLEN */
#define CMD17   (0x40+17)       /* READ_SINGLE_BLOCK */
#define CMD24   (0x40+24)       /* WRITE_BLOCK */
#define CMD55   (0x40+55)       /* APP_CMD */
#define CMD58   (0x40+58)       /* READ_OCR */

#define   SERCOM_MODE_USART_EXT_CLK_Val         0x0ul
#define   SERCOM_MODE_USART_INT_CLK_Val         0x1ul
#define   SERCOM_MODE_SPI_SLAVE_Val             0x2ul
#define   SERCOM_MODE_SPI_MASTER_Val            0x3ul
#define   SERCOM_MODE_I2C_SLAVE_Val             0x4ul
#define   SERCOM_MODE_I2C_MASTER_Val            0x5ul

#define   SERCOM_DORD_MSB_FIRST_Val             0x0ul
#define   SERCOM_DORD_LSB_FIRST_Val             0x1ul

#define   SERCOM_CHAR_SIZE_8_BITS_Val           0x0ul
#define   SERCOM_CHAR_SIZE_9_BITS_Val           0x1ul

#define   SDCARD_SPI_BUAD_SLOW                  250000ul
#define   SDCARD_SPI_BUAD_FAST                  6000000ul

#ifdef __cplusplus
}
#endif

#endif	/* _DISKIO_DEFINED */
