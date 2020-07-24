/**************************************************************************************
   INCLUDE
 **************************************************************************************/

#include "lzss.h"

#include <stdlib.h>
#include <stdint.h>

#include <MKRNB.h>
#include <FlashStorage.h>

/**************************************************************************************
   DEFINE
 **************************************************************************************/

#define EI 11             /* typically 10..13 */
#define EJ  4             /* typically 4..5 */
#define P   1             /* If match length <= P then output one character */
#define N (1 << EI)       /* buffer size */
#define F ((1 << EJ) + 1) /* lookahead buffer size */

#define LZSS_EOF       (-1)

#define FPUTC_BUF_SIZE (512)
#define FGETC_BUF_SIZE (512)

/**************************************************************************************
   GLOBAL VARIABLES
 **************************************************************************************/

extern NBFileUtils fileUtils;
extern FlashClass mcu_flash;
extern const char * UPDATE_FILE_NAME_LZSS;

static uint32_t SKETCH_START = 0;
static uint32_t LZSS_FILE_SIZE = 0;

int bit_buffer = 0, bit_mask = 128;
unsigned long codecount = 0, textcount = 0;
unsigned char buffer[N * 2];

static char write_buf[FPUTC_BUF_SIZE];
static size_t write_buf_num_bytes = 0;
static size_t bytes_written_fputc = 0;
static size_t bytes_written_flash = 0;
static uint32_t flash_addr = 0;

bool fromLZSStoBIN = true;
bool append = false;
bool endOfFile = false;

/**************************************************************************************
   PUBLIC FUNCTIONS
 **************************************************************************************/

void lzss_init(uint32_t const sketch_start, bool LZSStoBIN)
{
    fromLZSStoBIN = LZSStoBIN;
    if (LZSStoBIN) {
        SKETCH_START = sketch_start;
        flash_addr = sketch_start;
        LZSS_FILE_SIZE = fileUtils.listFile("UPDATE.BIN.LZSS");
    }
}

void lzss_flush()
{
  bytes_written_fputc += write_buf_num_bytes;

    if (fromLZSStoBIN) {
        /* Only write to the flash once we've surpassed
        * the SBU in the update binary.
        */
        if (bytes_written_fputc > (SKETCH_START - 0x2000))
        {
            mcu_flash.write((void*)flash_addr, write_buf, write_buf_num_bytes);
            flash_addr += write_buf_num_bytes;
        }
    } else {
        fileUtils.downloadFile("UPDATE.BIN.LZSS", write_buf, write_buf_num_bytes, append);
        append = true;
    }
  
  write_buf_num_bytes = 0;
}

/**************************************************************************************
   PRIVATE FUNCTIONS
 **************************************************************************************/

void lzss_fputc(int const c)
{
  /* Buffer the decompressed data into a buffer so
   * we can perform block writes and don't need to
   * write every byte singly on the flash (which 
   * wouldn't be possible anyway).
   */
  write_buf[write_buf_num_bytes] = static_cast<char>(c);
  write_buf_num_bytes++;

  /* The write buffer is full of decompressed
   * data, write it to the flash now.
   */
  if (write_buf_num_bytes == FPUTC_BUF_SIZE || endOfFile)
    lzss_flush();
}

/**************************************************************************************
   LZSS FUNCTIONS
 **************************************************************************************/

void putbit1(void)
{
    bit_buffer |= bit_mask;
    if ((bit_mask >>= 1) == 0) {
        lzss_fputc(bit_buffer);
        bit_buffer = 0;  bit_mask = 128;
    }
}

void putbit0(void)
{
    if ((bit_mask >>= 1) == 0) {
        lzss_fputc(bit_buffer);
        bit_buffer = 0;  bit_mask = 128;
    }
}

void flush_bit_buffer(void)
{
    if (bit_mask != 128) {
        //if (fputc(bit_buffer, outfile) == EOF) error();
        lzss_fputc(bit_buffer);
        codecount++;
    }
}

void output1(int c)
{
    int mask;
    
    putbit1();
    mask = 256;
    while (mask >>= 1) {
        if (c & mask) putbit1();
        else putbit0();
    }
}

void output2(int x, int y)
{
    int mask;
    
    putbit0();
    mask = N;
    while (mask >>= 1) {
        if (x & mask) putbit1();
        else putbit0();
    }
    mask = (1 << EJ);
    while (mask >>= 1) {
        if (y & mask) putbit1();
        else putbit0();
    }
}

int lzss_encode(const char buf_in[], uint32_t size)
{
    int i, j, f1, x, y, r, s, bufferend, c;
    
    for (i = 0; i < N - F; i++) buffer[i] = ' ';
    for (i = N - F; i < N * 2; i++) {
        if (textcount >= size) {
            endOfFile = true;
            break;
        } else {
            buffer[i] = buf_in[textcount];
            textcount++;
        }
    }
    bufferend = i;  r = N - F;  s = 0;
    while (r < bufferend) {
        f1 = (F <= bufferend - r) ? F : bufferend - r;
        x = 0;  y = 1;  c = buffer[r];
        for (i = r - 1; i >= s; i--)
            if (buffer[i] == c) {
                for (j = 1; j < f1; j++)
                    if (buffer[i + j] != buffer[r + j]) break;
                if (j > y) {
                    x = i;  y = j;
                }
            }
        if (y <= P) {  y = 1;  output1(c);  }
        else output2(x & (N - 1), y - 2);
        r += y;  s += y;
        if (r >= N * 2 - F) {
            for (i = 0; i < N; i++) buffer[i] = buffer[i + N];
            bufferend -= N;  r -= N;  s -= N;
            while (bufferend < N * 2) {
                if (textcount >= size) {
                    endOfFile = true;
                    break;
                } else {
                    buffer[bufferend++] = buf_in[textcount];
                    textcount++;
                }
            }
        }
    }
    flush_bit_buffer();

    return bytes_written_fputc;
}