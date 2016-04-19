#include <SPI.h>
#include "at86rf2xx-registers.h"
#define Serial SERIAL_PORT_USBVIRTUAL

#define MMIO_REG(mem_addr, type) (*(volatile type *)(mem_addr))
#define RFCTRL_FECTRL MMIO_REG(0x42005400, uint16_t)

// ANT_DIV
#define ANT_SEL           7
#define ANT_DIV_EN        3
#define ANT_EXT_SW_EN     2
#define ANT_CTRL          0

#define DEFAULT_CHANNEL 18
#define DEFAULT_ATTENUATION 0x07 //0dBm
#define DEFAULT_ANTENNA 2 //ANT1 = SMA, ANT2 = CHIP
#define DEFAULT_XTAL_TRIM 0x08 //midscale
#define DEFAULT_XTAL_MODE 0xf0

#define RF_CMD_REG_W      ((1<<7) | (1<<6))
#define RF_CMD_REG_R      ((1<<7) | (0<<6))
#define RF_CMD_FRAME_W    ((0<<7) | (1<<6) | (1<<5))
#define RF_CMD_FRAME_R    ((0<<7) | (0<<6) | (1<<5))
#define RF_CMD_SRAM_W     ((0<<7) | (1<<6) | (0<<5))
#define RF_CMD_SRAM_R     ((0<<7) | (0<<6) | (0<<5))

volatile int rf_events = 0;

byte part_number;

void setup() {
  // put your setup code here, to run once:

  while(!Serial);

  Serial.begin(9600);


  pinMode(PIN_RF1, OUTPUT);

  pinMode(PIN_RF2, OUTPUT);

//  PORT->Group[PORTA].PMUX[4].bit.PMUXO = PORT_PMUX_PMUXO_F_Val;
//  PORT->Group[PORTA].PMUX[6].bit.PMUXE = PORT_PMUX_PMUXE_F_Val;

  pinMode(PIN_ATRF233_RST, OUTPUT);
  pinMode(PIN_ATRF233_IRQ, INPUT);
  pinMode(PIN_ATRF233_SS, OUTPUT);
  pinMode(PIN_ATRF233_SLP_TR, OUTPUT);

//  pinMode(PIN_ATRF233_MOSI, OUTPUT);
//  pinMode(PIN_ATRF233_MISO, INPUT);
//  pinMode(PIN_ATRF233_SCK, OUTPUT);

  // Enable RFCTRL (AT86RF233 on SERCOM4)
//  PM->APBCMASK.reg |= (1 << PM_APBCMASK_RFCTRL_Pos);
  PM->APBCMASK.reg |= PM_APBCMASK_RFCTRL;
  
  RFCTRL_FECTRL = (0 << 4/*DIG1*/) | (1 << 2/*DIG2*/);
//  RFCTRL_FECTRL = (2 << 4/*DIG3*/) | (3 << 2/*DIG4*/);
//  REG_RFCTRL_FECFG = RFCTRL_FECFG_F1CFG(1)|RFCTRL_FECFG_F0CFG(0); // DIG2, DIG1

  attachInterrupt(digitalPinToInterrupt(PIN_ATRF233_IRQ), rf_irq_handler, RISING);
  
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  delay(10);

  
  // Wake up.
  digitalWrite(PIN_ATRF233_SLP_TR, LOW);
  delay( (300U) );

  // Reset
  digitalWrite(PIN_ATRF233_RST, LOW);
  delay( (1U) );
  digitalWrite(PIN_ATRF233_RST, HIGH);
  delay( (26U) );

  // Set channel
  rf_channel( DEFAULT_CHANNEL );
  
  // Set TX Power
  rf_write(AT86RF2XX_REG__PHY_TX_PWR, DEFAULT_ATTENUATION);
  
  // Set clock.
  rf_write(AT86RF2XX_REG__XOSC_CTRL, DEFAULT_XTAL_MODE | DEFAULT_XTAL_TRIM);
  
  // Control
  rf_write(AT86RF2XX_REG__TRX_CTRL_0, 0x01);
  rf_write(AT86RF2XX_REG__TRX_CTRL_1, 0x00);
  rf_write(AT86RF2XX_REG__TST_CTRL_DIGI, 0x0f);

  // Frame mode
  digitalWrite(PIN_ATRF233_SS, LOW);
  SPI.transfer(RF_CMD_FRAME_W);
  SPI.transfer(127);

  for (uint8_t i = 0; i <= 127; i++)
  {
    SPI.transfer((uint8_t) rand());
  }
  digitalWrite(PIN_ATRF233_SS, HIGH);
  
  // Set ID.
  rf_write(AT86RF2XX_REG__PART_NUM, 0x54);
  rf_write(AT86RF2XX_REG__PART_NUM, 0x46);

  part_number = rf_read(AT86RF2XX_REG__PART_NUM);
  Serial.print("Part is 0x");
  Serial.println(part_number, HEX);
  
  Serial.println("Turning on radio...");
  
  // Turn on, start
  rf_trx_set_state(AT86RF2XX_TRX_STATUS__PLL_ON);
  rf_write(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__TX_START);

  Serial.println("OK, ready.");
  
}

void loop() {
  // put your main code here, to run repeatedly:

  if (rf_events > 0) {
    
    Serial.print("rf event:");
    Serial.println(rf_events);
  }
}

static void rf_irq_handler()
{
    rf_events++;
    return;
}

static void rf_channel(uint8_t channel)
{
  uint8_t reg;
  reg = rf_read(AT86RF2XX_REG__PHY_CC_CCA) & ~0x1f;
  rf_write(AT86RF2XX_REG__PHY_CC_CCA, reg | channel);
}
static void rf_write(uint8_t addr, uint8_t value)
{
  byte write_command = AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_WRITE | addr;
  digitalWrite(PIN_ATRF233_SS, LOW);
  
  SPI.transfer(write_command);
  SPI.transfer(value);
  
  digitalWrite(PIN_ATRF233_SS, HIGH);
}

static uint8_t rf_read(uint8_t addr)
{
  byte read_command = AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_READ | addr;
  byte value;

  digitalWrite(PIN_ATRF233_SS, LOW);

  SPI.transfer(read_command);
  value = SPI.transfer(0x00);
  
  digitalWrite(PIN_ATRF233_SS, HIGH);

  return (uint8_t) value;
}

static void rf_trx_set_state(uint8_t state)
{
  rf_write(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__FORCE_TRX_OFF);
  rf_wait_state(AT86RF2XX_TRX_STATUS__TRX_OFF);

  rf_write(AT86RF2XX_REG__TRX_STATE, state);
  rf_wait_state(state);
}

static void rf_wait_state(uint8_t state)
{
  while (state != (rf_read (AT86RF2XX_REG__TRX_STATUS) & AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS))
    ;
}
