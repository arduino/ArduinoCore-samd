#!/bin/bash -ex

make clean

BOARD_ID=arduino_zero MCU=SAMD21G18A make all mostly_clean

BOARD_ID=genuino_zero MCU=SAMD21G18A make all mostly_clean

BOARD_ID=arduino_mkr1000 MCU=SAMD21G18A make all mostly_clean

BOARD_ID=genuino_mkr1000 MCU=SAMD21G18A make all mostly_clean

BOARD_ID=arduino_mkrzero MCU=SAMD21G18A make all mostly_clean

BOARD_ID=MT_D21E_rev_A MCU=SAMD21E17A make all mostly_clean
BOARD_ID=MT_D21E_rev_A MCU=SAMD21E18A make all mostly_clean

BOARD_ID=MT_D21E_rev_B MCU=SAMD21E17A make all mostly_clean
BOARD_ID=MT_D21E_rev_B MCU=SAMD21E18A make all mostly_clean
BOARD_ID=MT_D21E_rev_B MCU=SAML21E18B make all mostly_clean
BOARD_ID=MT_D21E_rev_B MCU=SAMC21E18A make all mostly_clean

BOARD_ID=MT_D11 MCU=SAMD11D14AM make all mostly_clean

BOARD_ID=Generic_x21E MCU=SAMD21E15A make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAMD21E16A make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAMD21E17A make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAMD21E18A make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAML21E15B make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAML21E16B make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAML21E17B make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAML21E18B make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAMC21E15A make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAMC21E16A make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAMC21E17A make all mostly_clean
BOARD_ID=Generic_x21E MCU=SAMC21E18A make all mostly_clean

BOARD_ID=Generic_x21G MCU=SAMD21G15A make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAMD21G16A make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAMD21G17A make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAMD21G18A make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAML21G16B make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAML21G17B make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAML21G18B make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAMC21G15A make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAMC21G16A make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAMC21G17A make all mostly_clean
BOARD_ID=Generic_x21G MCU=SAMC21G18A make all mostly_clean

BOARD_ID=Generic_x21J MCU=SAMD21J15A make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAMD21J16A make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAMD21J17A make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAMD21J18A make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAML21J16B make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAML21J17B make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAML21J18B make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAMC21J15A make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAMC21J16A make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAMC21J17A make all mostly_clean
BOARD_ID=Generic_x21J MCU=SAMC21J18A make all mostly_clean

BOARD_ID=Generic_D11D14AM MCU=SAMD11D14AM make all mostly_clean
BOARD_ID=Generic_D11D14AS MCU=SAMD11D14AS make all mostly_clean
BOARD_ID=Generic_D11C14A MCU=SAMD11C14A make all mostly_clean

BOARD_ID=arduino_m0 MCU=SAMD21G18A make all mostly_clean
BOARD_ID=arduino_m0_pro MCU=SAMD21G18A make all mostly_clean

mv -v *.bin ./binaries/

echo Done building bootloaders!

