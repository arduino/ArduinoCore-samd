/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

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

#include "variant.h"
#include "Arduino.h"

const PinDescription g_APinDescription[] = {

/*
 +------------+------------------+--------+-----------------+--------+-----------------------+---------+---------+--------+--------+----------+----------+
 | Pin number |  MKR  Board pin  |  PIN   | Notes           | Peri.A |     Peripheral B      | Perip.C | Perip.D | Peri.E | Peri.F | Periph.G | Periph.H |
 |            |                  |        |                 |   EIC  | ADC |  AC | PTC | DAC | SERCOMx | SERCOMx |  TCCx  |  TCCx  |    COM   | AC/GLCK  |
 |            |                  |        |                 |(EXTINT)|(AIN)|(AIN)|     |     | (x/PAD) | (x/PAD) | (x/WO) | (x/WO) |          |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 | 00         | D0               |  PA22  |                 |  *06   |     |     | X10 |     |   3/00  |   5/00  |* TC4/0 | TCC0/4 |          | GCLK_IO6 |
 | 01         | D1               |  PA23  |                 |  *07   |     |     | X11 |     |   3/01  |   5/01  |* TC4/1 | TCC0/5 | USB/SOF  | GCLK_IO7 |
 | 02         | D2               |  PA10  |                 |   10   | *18 |     | X02 |     |   0/02  |   2/02  |*TCC1/0 | TCC0/2 | I2S/SCK0 | GCLK_IO4 |
 | 03         | D3               |  PA11  |                 |   11   | *19 |     | X03 |     |   0/03  |   2/03  |*TCC1/1 | TCC0/3 | I2S/FS0  | GCLK_IO5 |
 | 04         | D4               |  PB10  |                 |  *10   |     |     |     |     |         |   4/02  |* TC5/0 | TCC0/4 | I2S/MCK1 | GCLK_IO4 |
 | 05         | D5               |  PB11  |                 |  *11   |     |     |     |     |         |   4/03  |* TC5/1 | TCC0/5 | I2S/SCK1 | GCLK_IO5 |
 | 06         | D6               |  PA20  | LED_BUILTIN     |  *04   |     |     | X08 |     |   5/02  |   3/02  |        |*TCC0/6 | I2S/SCK0 | GCLK_IO4 |
 | 07         | D7               |  PA21  |                 |  *05   |     |     | X09 |     |   5/03  |   3/03  |        |*TCC0/7 | I2S/FS0  | GCLK_IO5 |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 */
  { PORTA, 22, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), No_ADC_Channel, PWM4_CH0,   TC4_CH0,      EXTERNAL_INT_6    },
  { PORTA, 23, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), No_ADC_Channel, PWM4_CH1,   TC4_CH1,      EXTERNAL_INT_7    },
  { PORTA, 10, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), ADC_Channel18,  PWM1_CH0,   TCC1_CH0,     EXTERNAL_INT_NONE },
  { PORTA, 11, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), ADC_Channel19,  PWM1_CH1,   TCC1_CH1,     EXTERNAL_INT_NONE },
  { PORTB, 10, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), No_ADC_Channel, PWM5_CH0,   TC5_CH0,      EXTERNAL_INT_10   },
  { PORTB, 11, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), No_ADC_Channel, PWM5_CH1,   TC5_CH1,      EXTERNAL_INT_11   },
  { PORTA, 20, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH6,   TCC0_CH6,     EXTERNAL_INT_4    },
  { PORTA, 21, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH7,   TCC0_CH7,     EXTERNAL_INT_5    },

/*
 +------------+------------------+--------+-----------------+--------+-----------------------+---------+---------+--------+--------+----------+----------+
 | Pin number |  MKR  Board pin  |  PIN   | Notes           | Peri.A |     Peripheral B      | Perip.C | Perip.D | Peri.E | Peri.F | Periph.G | Periph.H |
 |            |                  |        |                 |   EIC  | ADC |  AC | PTC | DAC | SERCOMx | SERCOMx |  TCCx  |  TCCx  |    COM   | AC/GLCK  |
 |            |                  |        |                 |(EXTINT)|(AIN)|(AIN)|     |     | (x/PAD) | (x/PAD) | (x/WO) | (x/WO) |          |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 |            |       SPI        |        |                 |        |     |     |     |     |         |         |        |        |          |          |
 | 08         | MOSI             |  PA16  |                 |  *00   |     |     | X04 |     |  *1/00  |   3/00  |*TCC2/0 | TCC0/6 |          | GCLK_IO2 |
 | 09         | SCK              |  PA17  |                 |  *01   |     |     | X05 |     |  *1/01  |   3/01  | TCC2/1 | TCC0/7 |          | GCLK_IO3 |
 | 10         | MISO             |  PA19  |                 |   03   |     |     | X07 |     |  *1/03  |   3/03  |* TC3/1 | TCC0/3 | I2S/SD0  | AC/CMP1  |
 +------------+------------------+--------+-----------------+--------------------+-----+-----+---------+---------+--------+--------+----------+----------+
 |            |       Wire       |        |                 |        |     |     |     |     |         |         |        |        |          |          |
 | 11         | SDA              |  PA08  |                 |   NMI  | *16 |     | X00 |     |  *0/00  |   2/00  | TCC0/0 | TCC1/2 | I2S/SD1  |          |
 | 12         | SCL              |  PA09  |                 |   09   | *17 |     | X01 |     |  *0/01  |   2/01  | TCC0/1 | TCC1/3 | I2S/MCK0 |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 |            |      Serial1     |        |                 |        |     |     |     |     |         |         |        |        |          |          |
 | 13         | RX               |  PB23  |                 |   07   |     |     |     |     |         |  *5/03  |        |        |          | GCLK_IO1 |
 | 14         | TX               |  PB22  |                 |   06   |     |     |     |     |         |  *5/02  |        |        |          | GCLK_IO0 |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 */
                                                                                                                                               // DIPO=3 DOPO=0
  { PORTA, 16, PIO_SERCOM,  (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), No_ADC_Channel, PWM2_CH0,   TCC2_CH0,     EXTERNAL_INT_0    }, // MOSI: SERCOM1/PAD[0]
  { PORTA, 17, PIO_SERCOM,  (PIN_ATTR_DIGITAL                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, // SCK:  SERCOM1/PAD[1]
  { PORTA, 19, PIO_SERCOM,  (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), No_ADC_Channel, PWM3_CH1,   TC3_CH1,      EXTERNAL_INT_NONE }, // MISO: SERCOM1/PAD[3]

                                                                                                                                               // PINOUT=0
  { PORTA,  8, PIO_SERCOM_ALT,  (PIN_ATTR_DIGITAL                            ), ADC_Channel16,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI  }, // SDA:  SERCOM2/PAD[0]
  { PORTA,  9, PIO_SERCOM_ALT,  (PIN_ATTR_DIGITAL                            ), ADC_Channel17,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCL:  SERCOM2/PAD[1]

                                                                                                                                               // RXPO=3 TXPO=1
  { PORTB, 23, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL                             ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // RX:   SERCOM5/PAD[3]
  { PORTB, 22, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL                             ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // TX:   SERCOM5/PAD[2]


/*
 +------------+------------------+--------+-----------------+--------+-----------------------+---------+---------+--------+--------+----------+----------+
 | Pin number |  MKR  Board pin  |  PIN   | Notes           | Peri.A |     Peripheral B      | Perip.C | Perip.D | Peri.E | Peri.F | Periph.G | Periph.H |
 |            |                  |        |                 |   EIC  | ADC |  AC | PTC | DAC | SERCOMx | SERCOMx |  TCCx  |  TCCx  |    COM   | AC/GLCK  |
 |            |                  |        |                 |(EXTINT)|(AIN)|(AIN)|     |     | (x/PAD) | (x/PAD) | (x/WO) | (x/WO) |          |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 | 15         | A0 / DAC0        |  PA02  |                 |   02   | *00 |     | Y00 | OUT |         |         |        |        |          |          |
 | 16         | A1               |  PB02  |                 |  *02   | *10 |     | Y08 |     |         |   5/00  |        |        |          |          |
 | 17         | A2               |  PB03  |                 |  *03   | *11 |     | Y09 |     |         |   5/01  |        |        |          |          |
 | 18         | A3               |  PA04  |                 |   04   | *04 |  00 | Y02 |     |         |   0/00  |*TCC0/0 |        |          |          |
 | 19         | A4               |  PA05  |                 |   05   | *05 |  01 | Y03 |     |         |   0/01  |*TCC0/1 |        |          |          |
 | 20         | A5               |  PA06  |                 |   06   | *06 |  02 | Y04 |     |         |   0/02  | TCC1/0 |        |          |          |
 | 21         | A6               |  PA07  |                 |   07   | *07 |  03 | Y05 |     |         |   0/03  | TCC1/1 |        | I2S/SD0  |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 */
  { PORTA,  2, PIO_ANALOG,  (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG /*DAC*/        ), ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTB,  2, PIO_ANALOG,  (PIN_ATTR_DIGITAL                                ), ADC_Channel10,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    },
  { PORTB,  3, PIO_ANALOG,  (PIN_ATTR_DIGITAL                                ), ADC_Channel11,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    },
  { PORTA,  4, PIO_ANALOG,  (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), ADC_Channel4,   PWM0_CH0,   TCC0_CH0,     EXTERNAL_INT_NONE },
  { PORTA,  5, PIO_ANALOG,  (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER    ), ADC_Channel5,   PWM0_CH1,   TCC0_CH1,     EXTERNAL_INT_NONE },
  { PORTA,  6, PIO_ANALOG,  (PIN_ATTR_DIGITAL                                ), ADC_Channel6,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA,  7, PIO_ANALOG,  (PIN_ATTR_DIGITAL                                ), ADC_Channel7,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },

/*
 +------------+------------------+--------+-----------------+--------+-----------------------+---------+---------+--------+--------+----------+----------+
 | Pin number |  MKR  Board pin  |  PIN   | Notes           | Peri.A |     Peripheral B      | Perip.C | Perip.D | Peri.E | Peri.F | Periph.G | Periph.H |
 |            |                  |        |                 |   EIC  | ADC |  AC | PTC | DAC | SERCOMx | SERCOMx |  TCCx  |  TCCx  |    COM   | AC/GLCK  |
 |            |                  |        |                 |(EXTINT)|(AIN)|(AIN)|     |     | (x/PAD) | (x/PAD) | (x/WO) | (x/WO) |          |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 |            |       USB        |        |                 |        |     |     |     |     |         |         |        |        |          |          |
 | 22         |                  |  PA24  | USB N           |   12   |     |     |     |     |   3/02  |   5/02  |  TC5/0 | TCC1/2 | USB/DM   |          |
 | 23         |                  |  PA25  | USB P           |   13   |     |     |     |     |   3/03  |   5/03  |  TC5/1 | TCC1/3 | USB/DP   |          |
 | 24         |                  |  PA18  | USB ID          |   02   |     |     | X06 |     |   1/02  |   3/02  |  TC3/0 | TCC0/2 |          | AC/CMP0  |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 | 25         | AREF             |  PA03  |                 |   03   |  01 |     | Y01 |     |         |         |        |        |          |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 */
  { PORTA, 24, PIO_COM,     (PIN_ATTR_NONE                                   ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM,     (PIN_ATTR_NONE                                   ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP
  { PORTA, 18, PIO_DIGITAL, (PIN_ATTR_DIGITAL                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },

  { PORTA,  3, PIO_DIGITAL, (PIN_ATTR_DIGITAL                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

/*
 +------------+------------------+--------+-----------------+--------+-----------------------+---------+---------+--------+--------+----------+----------+
 | Pin number |  MKR  Board pin  |  PIN   | Notes           | Peri.A |     Peripheral B      | Perip.C | Perip.D | Peri.E | Peri.F | Periph.G | Periph.H |
 |            |                  |        |                 |   EIC  | ADC |  AC | PTC | DAC | SERCOMx | SERCOMx |  TCCx  |  TCCx  |    COM   | AC/GLCK  |
 |            |                  |        |                 |(EXTINT)|(AIN)|(AIN)|     |     | (x/PAD) | (x/PAD) | (x/WO) | (x/WO) |          |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 |            | WiFi SPI         |        |                 |        |     |     |     |     |         |         |        |        |          |          |
 | 26         |                  |  PA12  | NINA_MOSI       |   12   |     |     |     |     |  *2/00  |   4/00  | TCC2/0 | TCC0/6 |          | AC/CMP0  |
 | 27         |                  |  PA13  | NINA_MISO       |   13   |     |     |     |     |  *2/01  |   4/01  | TCC2/1 | TCC0/7 |          | AC/CMP1  |
 | 28         |                  |  PA14  | NINA_CS         |   14   |     |     |     |     |   2/02  |   4/02  |  TC3/0 | TCC0/4 |          | GCLK_IO0 |
 | 29         |                  |  PA15  | NINA_SCK        |   15   |     |     |     |     |  *2/03  |   4/03  |  TC3/1 | TCC0/5 |          | GCLK_IO1 |
 | 30         |                  |  PA27  | NINA_GPIO0      |  *15   |     |     |     |     |         |         |        |        |          | GCLK_IO0 |
 | 31         |                  |  PB08  | NINA_RESETN     |   08   |  02 |     | Y14 |     |         |   4/00  |  TC4/0 |        |          |          |
 | 32         |                  |  PB09  | ADC_VBAT        |  *09   |  03 |     | Y15 |     |         |   4/01  |  TC4/1 |        |          |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 |            | 32768Hz Crystal  |        |                 |        |     |     |     |     |         |         |        |        |          |          |
 | 33         |                  |  PA00  | XIN32           |   00   |     |     |     |     |         |   1/00  | TCC2/0 |        |          |          |
 | 34         |                  |  PA01  | XOUT32          |   01   |     |     |     |     |         |   1/01  | TCC2/1 |        |          |          |
 +------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
 | 35         |                  |  PA28  | NINA_ACK        |   01   |     |     |     |     |         |   1/01  | TCC2/1 |        |          |          |
+------------+------------------+--------+-----------------+--------+-----+-----+-----+-----+---------+---------+--------+--------+----------+----------+
*/
                                                                                                                                               // DIPO=3 DOPO=0
  { PORTA, 12, PIO_SERCOM_ALT, (PIN_ATTR_NONE                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MOSI: SERCOM4/PAD[0]
  { PORTA, 13, PIO_SERCOM_ALT, (PIN_ATTR_NONE                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCK:  SERCOM4/PAD[1]
  { PORTA, 14, PIO_DIGITAL,    (PIN_ATTR_NONE                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SS:   as GPIO
  { PORTA, 15, PIO_SERCOM_ALT, (PIN_ATTR_NONE                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MISO: SERCOM4/PAD[3]
  { PORTA, 27, PIO_DIGITAL,    (PIN_ATTR_NONE                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },
  
  { PORTB,  8, PIO_DIGITAL,    (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG             ), ADC_Channel2,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTB,  9, PIO_ANALOG,     (PIN_ATTR_PWM|PIN_ATTR_TIMER                  ), ADC_Channel3,   PWM4_CH1,   TC4_CH1,      EXTERNAL_INT_9    },

  { PORTA,  0, PIO_DIGITAL,    (PIN_ATTR_NONE                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA,  1, PIO_DIGITAL,    (PIN_ATTR_NONE                                ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },

  { PORTA, 28, PIO_DIGITAL,    (PIN_ATTR_DIGITAL                             ), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
};

extern "C" {
    unsigned int PINCOUNT_fn() {
        return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
    }
}

const void* g_apTCInstances[TCC_INST_NUM + TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 };

// Multi-serial objects instantiation
SERCOM sercom0(SERCOM0);
SERCOM sercom1(SERCOM1);
SERCOM sercom2(SERCOM2);
SERCOM sercom3(SERCOM3);
SERCOM sercom4(SERCOM4);
SERCOM sercom5(SERCOM5);

#if defined(USE_BQ24195L_PMIC)

#include "wiring_private.h"

#define PMIC_ADDRESS  0x6B
#define PMIC_REG01    0x01
#define PMIC_REG07    0x07

static inline void enable_battery_charging() {
  PERIPH_WIRE.initMasterWIRE(100000);
  PERIPH_WIRE.enableWIRE();
  pinPeripheral(PIN_WIRE_SDA, g_APinDescription[PIN_WIRE_SDA].ulPinType);
  pinPeripheral(PIN_WIRE_SCL, g_APinDescription[PIN_WIRE_SCL].ulPinType);

  PERIPH_WIRE.startTransmissionWIRE( PMIC_ADDRESS, WIRE_WRITE_FLAG );
  PERIPH_WIRE.sendDataMasterWIRE(PMIC_REG01);
  PERIPH_WIRE.sendDataMasterWIRE(0x1B); // Charge Battery + Minimum System Voltage 3.5V
  PERIPH_WIRE.prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);

  PERIPH_WIRE.disableWIRE();
}

static inline void disable_battery_fet(bool disabled) {
  PERIPH_WIRE.initMasterWIRE(100000);
  PERIPH_WIRE.enableWIRE();
  pinPeripheral(PIN_WIRE_SDA, g_APinDescription[PIN_WIRE_SDA].ulPinType);
  pinPeripheral(PIN_WIRE_SCL, g_APinDescription[PIN_WIRE_SCL].ulPinType);

  PERIPH_WIRE.startTransmissionWIRE( PMIC_ADDRESS, WIRE_WRITE_FLAG );
  PERIPH_WIRE.sendDataMasterWIRE(PMIC_REG07);
  // No D+/D– detection + Safety timer not slowed by 2X during input DPM or thermal regulation +
  // BAT fet disabled/enabled + charge and bat fault INT
  PERIPH_WIRE.sendDataMasterWIRE(0x0B | (disabled ? 0x20 : 0x00));
  PERIPH_WIRE.prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);

  PERIPH_WIRE.disableWIRE();
}

#endif

void initVariant() {
#if defined(USE_BQ24195L_PMIC)
  pinMode(ADC_BATTERY, OUTPUT);
  digitalWrite(ADC_BATTERY, LOW);
  delay(10);
  pinMode(ADC_BATTERY, INPUT);
  delay(100);

  bool batteryPresent = analogRead(ADC_BATTERY) > 600;
  if (batteryPresent) {
    enable_battery_charging();
  }
  disable_battery_fet(!batteryPresent);
#endif

  // NINA - SPI boot
  pinMode(NINA_GPIO0, OUTPUT);
  digitalWrite(NINA_GPIO0, HIGH);

  // disable NINA
  pinMode(NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, HIGH);
}

// Serial1
Uart Serial1(&sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX);

void SERCOM5_Handler()
{
  Serial1.IrqHandler();
}

// Serial2
Uart Serial2(&sercom4, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX, PIN_SERIAL2_RTS, PIN_SERIAL2_CTS);

void SERCOM4_Handler()
{
  Serial2.IrqHandler();
}
