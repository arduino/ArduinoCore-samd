#include "variant.h"

void clockout(uint32_t gclk, int32_t divisor)
{
    GCLK_GENDIV_Type gendiv =
    {
        .bit.DIV = divisor,      // divider, linear or 2^(.DIV+1)
        .bit.ID  = gclk,         // GCLK_GENERATOR_"gclk"
    };
    GCLK->GENDIV.reg = gendiv.reg;

    // setup Clock Generator
    GCLK_GENCTRL_Type genctrl =
    {
        .bit.RUNSTDBY = 0,        // Run in Standby
        .bit.DIVSEL = 0,          // .DIV (above) Selection: 0=linear 1=powers of 2
        .bit.OE = 1,              // Output Enable to observe on any port pin capable of outputting GCLK_IO["gclk"]
        .bit.OOV = 0,             // Output Off Value
        .bit.IDC = 1,             // Improve Duty Cycle
        .bit.GENEN = 1,           // enable this GCLK
        //.bit.SRC = GCLK_SOURCE_OSC8M,
        .bit.SRC = GCLK_SOURCE_DFLL48M, // select GCLK source
        .bit.ID = gclk,           // output GCLK_GENERATOR_"gclk" on GCLK_IO["gclk"]
    };
    GCLK->GENCTRL.reg = genctrl.reg;
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
    {
        /* Wait for synchronization */
    }
}
