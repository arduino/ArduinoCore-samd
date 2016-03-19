/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */
#ifndef _SAM_INCLUDED_
#define _SAM_INCLUDED_

#define part_is_defined(part) (defined(__ ## part ## __))

/*
 * ----------------------------------------------------------------------------
 * SAML family
 * ----------------------------------------------------------------------------
 */

/* SAML21 series */
#define SAML21G18 ( \
    part_is_defined( SAML21G18A ) )

#define SAML21G16 ( \
    part_is_defined( SAML21G16A ) )

#define SAML21G17 ( \
    part_is_defined( SAML21G17A ) )

#define SAML21J18 ( \
    part_is_defined( SAML21J18A ) )

#define SAML21J17 ( \
    part_is_defined( SAML21J17A ) )

#define SAML21J16 ( \
    part_is_defined( SAML21J16A ) )

#define SAML21E15 ( \
    part_is_defined( SAML21E15A ) )

#define SAML21E16 ( \
    part_is_defined( SAML21E16A ) )

#define SAML21E17 ( \
    part_is_defined( SAML21E17A ) )

/* Entire SAML21 series */
#define SAML21_SERIES (SAML21G18 || SAML21G16 || SAML21G17 || SAML21J18 || SAML21J17 || SAML21J16 || SAML21E15 || SAML21E16 || SAML21E17)

/* Entire SAML family */
#define SAML_SERIES (SAML21_SERIES)

/*
 * ----------------------------------------------------------------------------
 * SAMG family
 * ----------------------------------------------------------------------------
 */

/* SAMG51 series */
#define SAMG51N1 ( \
    part_is_defined( SAMG51N18 ) )

#define SAMG51G1 ( \
    part_is_defined( SAMG51G18 ) )

/* Entire SAMG51 series */
#define SAMG51_SERIES (SAMG51N1 || SAMG51G1)

/* SAMG53 series */
#define SAMG53N1 ( \
    part_is_defined( SAMG53N19 ) )

#define SAMG53G1 ( \
    part_is_defined( SAMG53G19 ) )

/* Entire SAMG53 series */
#define SAMG53_SERIES (SAMG53N1 || SAMG53G1)

/* SAMG54 series */
#define SAMG54N1 ( \
    part_is_defined( SAMG54N19 ) )

#define SAMG54J1 ( \
    part_is_defined( SAMG54J19 ) )

#define SAMG54G1 ( \
    part_is_defined( SAMG54G19 ) )

/* Entire SAMG54 series */
#define SAMG54_SERIES (SAMG54N1 || SAMG54J1 || SAMG54G1)

/* Entire SAMG family */
#define SAMG_SERIES (SAMG53_SERIES || SAMG54_SERIES || SAMG51_SERIES)

/*
 * ----------------------------------------------------------------------------
 * SAMD family
 * ----------------------------------------------------------------------------
 */

/* SAMD11 series */
#define SAMD11C13 ( \
    part_is_defined( SAMD11C13A ) )

#define SAMD11C14 ( \
    part_is_defined( SAMD11C14A ) )

#define SAMD11D14A ( \
    part_is_defined( SAMD11D14AS ) || \
    part_is_defined( SAMD11D14AM ) )

#define SAMD11D13A ( \
    part_is_defined( SAMD11D13AM ) || \
    part_is_defined( SAMD11D13AS ) )

/* Entire SAMD11 series */
#define SAMD11_SERIES (SAMD11C13 || SAMD11C14 || SAMD11D14A || SAMD11D13A)

/* SAMD09 series */
#define SAMD09C13 ( \
    part_is_defined( SAMD09C13A ) )

/* Entire SAMD09 series */
#define SAMD09_SERIES (SAMD09C13)


#define SAMD10C14 ( \
    part_is_defined( SAMD10C14A ) )

#define SAMD10C13 ( \
    part_is_defined( SAMD10C13A ) )

#define SAMD10D13A ( \
    part_is_defined( SAMD10D13AS ) || \
    part_is_defined( SAMD10D13AM ) )

#define SAMD10D14A ( \
    part_is_defined( SAMD10D14AS ) || \
    part_is_defined( SAMD10D14AM ) )

/* Entire SAMD10 series */
#define SAMD10_SERIES (SAMD10C14 || SAMD10C13 || SAMD10D13A || SAMD10D14A)

/* SAMD21 series */
#define SAMD21J17 ( \
    part_is_defined( SAMD21J17A ) )

#define SAMD21E14 ( \
    part_is_defined( SAMD21E14A ) )

#define SAMD21E15 ( \
    part_is_defined( SAMD21E15A ) || \
    part_is_defined( SAMD21E15L ) )

#define SAMD21E16 ( \
    part_is_defined( SAMD21E16A ) || \
    part_is_defined( SAMD21E16L ) )

#define SAMD21E17 ( \
    part_is_defined( SAMD21E17A ) )

#define SAMD21E18 ( \
    part_is_defined( SAMD21E18A ) )

#define SAMD21J16 ( \
    part_is_defined( SAMD21J16A ) )

#define SAMD21J15 ( \
    part_is_defined( SAMD21J15A ) )

#define SAMD21G16 ( \
    part_is_defined( SAMD21G16A ) )

#define SAMD21G17 ( \
    part_is_defined( SAMD21G17A ) )

#define SAMD21G14 ( \
    part_is_defined( SAMD21G14A ) )

#define SAMD21G15 ( \
    part_is_defined( SAMD21G15A ) )

#define SAMD21J14 ( \
    part_is_defined( SAMD21J14A ) )

#define SAMD21G18 ( \
    part_is_defined( SAMD21G18A ) )

#define SAMD21J18 ( \
    part_is_defined( SAMD21J18A ) )

/* Entire SAMD21 series */
#define SAMD21_SERIES (SAMD21J17 || SAMD21E14 || SAMD21E15 || SAMD21E16 || SAMD21E17 || SAMD21E18 || SAMD21J16 || SAMD21J15 || SAMD21G16 || SAMD21G17 || SAMD21G14 || SAMD21G15 || SAMD21J14 || SAMD21G18 || SAMD21J18)

/* SAMD20 series */
#define SAMD20G18 ( \
    part_is_defined( SAMD20G18U ) )

#define SAMD20J1 ( \
    part_is_defined( SAMD20J14 ) || \
    part_is_defined( SAMD20J15 ) || \
    part_is_defined( SAMD20J16 ) || \
    part_is_defined( SAMD20J17 ) || \
    part_is_defined( SAMD20J18 ) )

#define SAMD20G1 ( \
    part_is_defined( SAMD20G14 ) || \
    part_is_defined( SAMD20G15 ) || \
    part_is_defined( SAMD20G16 ) || \
    part_is_defined( SAMD20G17 ) || \
    part_is_defined( SAMD20G18 ) )

#define SAMD20G17 ( \
    part_is_defined( SAMD20G17U ) )

#define SAMD20E1 ( \
    part_is_defined( SAMD20E14 ) || \
    part_is_defined( SAMD20E15 ) || \
    part_is_defined( SAMD20E16 ) || \
    part_is_defined( SAMD20E17 ) || \
    part_is_defined( SAMD20E18 ) || \
    part_is_defined( SAMD20E1F ) )

/* Entire SAMD20 series */
#define SAMD20_SERIES (SAMD20G18 || SAMD20J1 || SAMD20G1 || SAMD20G17 || SAMD20E1)

/* Entire SAMD family */
#define SAMD_SERIES (SAMD11_SERIES || SAMD09_SERIES || SAMD21_SERIES || SAMD20_SERIES || SAMD10_SERIES)

/*
 * ----------------------------------------------------------------------------
 * SAM3 family
 * ----------------------------------------------------------------------------
 */

/* SAM3U series */
#define SAM3U1 ( \
    part_is_defined( SAM3U1E ) || \
    part_is_defined( SAM3U1C ) )

#define SAM3U2 ( \
    part_is_defined( SAM3U2E ) || \
    part_is_defined( SAM3U2C ) )

#define SAM3U4 ( \
    part_is_defined( SAM3U4E ) || \
    part_is_defined( SAM3U4C ) )

/* Entire SAM3U series */
#define SAM3U_SERIES (SAM3U1 || SAM3U2 || SAM3U4)

/* SAM3S series */
#define SAM3S2 ( \
    part_is_defined( SAM3S2C ) || \
    part_is_defined( SAM3S2B ) || \
    part_is_defined( SAM3S2A ) )

#define SAM3S1 ( \
    part_is_defined( SAM3S1C ) || \
    part_is_defined( SAM3S1B ) || \
    part_is_defined( SAM3S1A ) )

#define SAM3S4 ( \
    part_is_defined( SAM3S4C ) || \
    part_is_defined( SAM3S4B ) || \
    part_is_defined( SAM3S4A ) )

/* Entire SAM3S series */
#define SAM3S_SERIES (SAM3S2 || SAM3S1 || SAM3S4)

/* SAM3XA series */
#define SAM3A4 ( \
    part_is_defined( SAM3A4C ) )

#define SAM3X4 ( \
    part_is_defined( SAM3X4C ) || \
    part_is_defined( SAM3X4E ) )

#define SAM3A8 ( \
    part_is_defined( SAM3A8C ) )

#define SAM3X8 ( \
    part_is_defined( SAM3X8C ) || \
    part_is_defined( SAM3X8E ) || \
    part_is_defined( SAM3X8H ) )

/* Entire SAM3XA series */
#define SAM3XA_SERIES (SAM3A4 || SAM3X4 || SAM3A8 || SAM3X8)

/* SAM3SD8 series */
#define SAM3S8 ( \
    part_is_defined( SAM3S8C ) || \
    part_is_defined( SAM3S8B ) )

#define SAM3SD8 ( \
    part_is_defined( SAM3SD8C ) || \
    part_is_defined( SAM3SD8B ) )

/* Entire SAM3SD8 series */
#define SAM3SD8_SERIES (SAM3S8 || SAM3SD8)

/* SAM3N series */
#define SAM3N1 ( \
    part_is_defined( SAM3N1C ) || \
    part_is_defined( SAM3N1B ) || \
    part_is_defined( SAM3N1A ) )

#define SAM3N0 ( \
    part_is_defined( SAM3N0C ) || \
    part_is_defined( SAM3N0B ) || \
    part_is_defined( SAM3N0A ) )

#define SAM3N00 ( \
    part_is_defined( SAM3N00B ) || \
    part_is_defined( SAM3N00A ) )

#define SAM3N2 ( \
    part_is_defined( SAM3N2C ) || \
    part_is_defined( SAM3N2B ) || \
    part_is_defined( SAM3N2A ) )

#define SAM3N4 ( \
    part_is_defined( SAM3N4C ) || \
    part_is_defined( SAM3N4B ) || \
    part_is_defined( SAM3N4A ) )

/* Entire SAM3N series */
#define SAM3N_SERIES (SAM3N1 || SAM3N0 || SAM3N00 || SAM3N2 || SAM3N4)

/* Entire SAM3 family */
#define SAM3_SERIES (SAM3S_SERIES || SAM3N_SERIES || SAM3U_SERIES || SAM3XA_SERIES || SAM3SD8_SERIES)

/*
 * ----------------------------------------------------------------------------
 * SAMR family
 * ----------------------------------------------------------------------------
 */

/* SAMR21 series */
#define SAMR21E16 ( \
    part_is_defined( SAMR21E16A ) )

#define SAMR21E18 ( \
    part_is_defined( SAMR21E18A ) )

#define SAMR21G16 ( \
    part_is_defined( SAMR21G16A ) )

#define SAMR21G17 ( \
    part_is_defined( SAMR21G17A ) )

#define SAMR21G18 ( \
    part_is_defined( SAMR21G18A ) )

#define SAMR21E17 ( \
    part_is_defined( SAMR21E17A ) )

/* Entire SAMR21 series */
#define SAMR21_SERIES (SAMR21E16 || SAMR21E18 || SAMR21G16 || SAMR21G17 || SAMR21G18 || SAMR21E17)

/* Entire SAMR family */
#define SAMR_SERIES (SAMR21_SERIES)

/*
 * ----------------------------------------------------------------------------
 * SAM4 family
 * ----------------------------------------------------------------------------
 */

/* SAM4CP series */
#define SAM4CP16B_ ( \
    part_is_defined( SAM4CP16B_0 ) || \
    part_is_defined( SAM4CP16B_1 ) )

/* Entire SAM4CP series */
#define SAM4CP_SERIES (SAM4CP16B_)

/* SAM4N series */
#define SAM4N8 ( \
    part_is_defined( SAM4N8A ) || \
    part_is_defined( SAM4N8B ) || \
    part_is_defined( SAM4N8C ) )

#define SAM4N16 ( \
    part_is_defined( SAM4N16B ) || \
    part_is_defined( SAM4N16C ) )

/* Entire SAM4N series */
#define SAM4N_SERIES (SAM4N8 || SAM4N16)

/* SAM4S series */
#define SAM4S8 ( \
    part_is_defined( SAM4S8C ) || \
    part_is_defined( SAM4S8B ) )

#define SAM4S2 ( \
    part_is_defined( SAM4S2A ) || \
    part_is_defined( SAM4S2B ) || \
    part_is_defined( SAM4S2C ) )

#define SAM4S4 ( \
    part_is_defined( SAM4S4A ) || \
    part_is_defined( SAM4S4B ) || \
    part_is_defined( SAM4S4C ) )

#define SAM4SD16 ( \
    part_is_defined( SAM4SD16B ) || \
    part_is_defined( SAM4SD16C ) )

#define SAM4S16 ( \
    part_is_defined( SAM4S16C ) || \
    part_is_defined( SAM4S16B ) )

#define SAM4SD32 ( \
    part_is_defined( SAM4SD32B ) || \
    part_is_defined( SAM4SD32C ) )

#define SAM4SA16 ( \
    part_is_defined( SAM4SA16B ) || \
    part_is_defined( SAM4SA16C ) )

/* Entire SAM4S series */
#define SAM4S_SERIES (SAM4S8 || SAM4S2 || SAM4S4 || SAM4SD16 || SAM4S16 || SAM4SD32 || SAM4SA16)

/* SAM4SP series */
#define SAM4SP32 ( \
    part_is_defined( SAM4SP32A ) )

/* Entire SAM4SP series */
#define SAM4SP_SERIES (SAM4SP32)

/* SAM4E series */
#define SAM4E16 ( \
    part_is_defined( SAM4E16E ) || \
    part_is_defined( SAM4E16C ) )

#define SAM4E8 ( \
    part_is_defined( SAM4E8E ) || \
    part_is_defined( SAM4E8C ) )

/* Entire SAM4E series */
#define SAM4E_SERIES (SAM4E16 || SAM4E8)

/* SAM4C series */
#define SAM4C8C_ ( \
    part_is_defined( SAM4C8C_0 ) || \
    part_is_defined( SAM4C8C_1 ) )

#define SAM4C16C_ ( \
    part_is_defined( SAM4C16C_0 ) || \
    part_is_defined( SAM4C16C_1 ) )

/* Entire SAM4C series */
#define SAM4C_SERIES (SAM4C8C_ || SAM4C16C_)

/* SAM4L series */
#define SAM4LS4 ( \
    part_is_defined( ATSAM4LS4A ) ||     part_is_defined( SAM4LS4A ) || \
    part_is_defined( ATSAM4LS4B ) ||     part_is_defined( SAM4LS4B ) || \
    part_is_defined( ATSAM4LS4C ) ||     part_is_defined( SAM4LS4C ) )

#define SAM4LS2 ( \
    part_is_defined( ATSAM4LS2A ) ||     part_is_defined( SAM4LS2A ) || \
    part_is_defined( ATSAM4LS2B ) ||     part_is_defined( SAM4LS2B ) || \
    part_is_defined( ATSAM4LS2C ) ||     part_is_defined( SAM4LS2C ) )

#define SAM4LS8 ( \
    part_is_defined( ATSAM4LS8A ) ||     part_is_defined( SAM4LS8A ) || \
    part_is_defined( ATSAM4LS8B ) ||     part_is_defined( SAM4LS8B ) || \
    part_is_defined( ATSAM4LS8C ) ||     part_is_defined( SAM4LS8C ) )

#define SAM4LC4 ( \
    part_is_defined( ATSAM4LC4A ) ||     part_is_defined( SAM4LC4A ) || \
    part_is_defined( ATSAM4LC4B ) ||     part_is_defined( SAM4LC4B ) || \
    part_is_defined( ATSAM4LC4C ) ||     part_is_defined( SAM4LC4C ) )

#define SAM4LC2 ( \
    part_is_defined( ATSAM4LC2A ) ||     part_is_defined( SAM4LC2A ) || \
    part_is_defined( ATSAM4LC2B ) ||     part_is_defined( SAM4LC2B ) || \
    part_is_defined( ATSAM4LC2C ) ||     part_is_defined( SAM4LC2C ) )

#define SAM4LC8 ( \
    part_is_defined( ATSAM4LC8A ) ||     part_is_defined( SAM4LC8A ) || \
    part_is_defined( ATSAM4LC8B ) ||     part_is_defined( SAM4LC8B ) || \
    part_is_defined( ATSAM4LC8C ) ||     part_is_defined( SAM4LC8C ) )

/* Entire SAM4L series */
#define SAM4L_SERIES (SAM4LS4 || SAM4LS2 || SAM4LS8 || SAM4LC4 || SAM4LC2 || SAM4LC8)

/* SAM4C32 series */
#define SAM4C32C_ ( \
    part_is_defined( SAM4C32C_0 ) || \
    part_is_defined( SAM4C32C_1 ) )

#define SAM4C32E_ ( \
    part_is_defined( SAM4C32E_0 ) || \
    part_is_defined( SAM4C32E_1 ) )

/* Entire SAM4C32 series */
#define SAM4C32_SERIES (SAM4C32C_ || SAM4C32E_)

/* SAM4CM32 series */
#define SAM4CMP32C_ ( \
    part_is_defined( SAM4CMP32C_0 ) || \
    part_is_defined( SAM4CMP32C_1 ) )

#define SAM4CMS32C_ ( \
    part_is_defined( SAM4CMS32C_0 ) || \
    part_is_defined( SAM4CMS32C_1 ) )

/* Entire SAM4CM32 series */
#define SAM4CM32_SERIES (SAM4CMP32C_ || SAM4CMS32C_)

/* SAM4CM series */
#define SAM4CMP16C_ ( \
    part_is_defined( SAM4CMP16C_0 ) || \
    part_is_defined( SAM4CMP16C_1 ) )

#define SAM4CMS16C_ ( \
    part_is_defined( SAM4CMS16C_0 ) || \
    part_is_defined( SAM4CMS16C_1 ) )

#define SAM4CMP8C_ ( \
    part_is_defined( SAM4CMP8C_0 ) || \
    part_is_defined( SAM4CMP8C_1 ) )

#define SAM4CMS8C_ ( \
    part_is_defined( SAM4CMS8C_0 ) || \
    part_is_defined( SAM4CMS8C_1 ) )

/* Entire SAM4CM series */
#define SAM4CM_SERIES (SAM4CMP16C_ || SAM4CMS16C_ || SAM4CMP8C_ || SAM4CMS8C_)

/* Entire SAM4 family */
#define SAM4_SERIES (SAM4E_SERIES || SAM4CM32_SERIES || SAM4SP_SERIES || SAM4C32_SERIES || SAM4S_SERIES || SAM4L_SERIES || SAM4CM_SERIES || SAM4N_SERIES || SAM4CP_SERIES || SAM4C_SERIES)

/*
 * ----------------------------------------------------------------------------
 * Whole SAM product line
 * ----------------------------------------------------------------------------
 */

#define SAM (SAM4_SERIES || SAMR_SERIES || SAM3_SERIES || WISC_SERIES || SAML_SERIES || SAMD_SERIES || SAMG_SERIES)

/*
 * ----------------------------------------------------------------------------
 * Header inclusion
 * ----------------------------------------------------------------------------
 */

#if SAML_SERIES
#include "saml.h"
#endif /* SAML_SERIES */

#if SAMG_SERIES
#include "samg.h"
#endif /* SAMG_SERIES */

#if SAMD_SERIES
#include "samd.h"
#endif /* SAMD_SERIES */

#if SAM3_SERIES
#include "sam3.h"
#endif /* SAM3_SERIES */

#if SAMR_SERIES
#include "samr.h"
#endif /* SAMR_SERIES */

#if SAM4_SERIES
#include "sam4.h"
#endif /* SAM4_SERIES */

#endif

