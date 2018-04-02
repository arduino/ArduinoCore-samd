/*
 * SAMD21_USBDevice.cpp
 *
 *  Created on: Feb 21, 2018
 *      Author: deanm
 */


#include "SAMD21_USBDevice.h"

void USBDevice_SAMD21G18x::reset() {
    usb.CTRLA.bit.SWRST = 1;
    memset(EP, 0, sizeof(EP));
    while (usb.SYNCBUSY.bit.SWRST || usb.SYNCBUSY.bit.ENABLE) {}
    usb.DESCADD.reg = (uint32_t)(&EP);
}

void USBDevice_SAMD21G18x::calibrate() {
    // Load Pad Calibration data from non-volatile memory
    uint32_t *pad_transn_p = (uint32_t *) USB_FUSES_TRANSN_ADDR;
    uint32_t *pad_transp_p = (uint32_t *) USB_FUSES_TRANSP_ADDR;
    uint32_t *pad_trim_p   = (uint32_t *) USB_FUSES_TRIM_ADDR;

    uint32_t pad_transn = (*pad_transn_p & USB_FUSES_TRANSN_Msk) >> USB_FUSES_TRANSN_Pos;
    uint32_t pad_transp = (*pad_transp_p & USB_FUSES_TRANSP_Msk) >> USB_FUSES_TRANSP_Pos;
    uint32_t pad_trim   = (*pad_trim_p   & USB_FUSES_TRIM_Msk  ) >> USB_FUSES_TRIM_Pos;

    if (pad_transn == 0x1F)  // maximum value (31)
        pad_transn = 5;
    if (pad_transp == 0x1F)  // maximum value (31)
        pad_transp = 29;
    if (pad_trim == 0x7)     // maximum value (7)
        pad_trim = 3;

    usb.PADCAL.bit.TRANSN = pad_transn;
    usb.PADCAL.bit.TRANSP = pad_transp;
    usb.PADCAL.bit.TRIM   = pad_trim;
}
