/* --COPYRIGHT--,BSD
 * Copyright (c; 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION; HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#ifndef ADS1258_H_
#define ADS1258_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

//*****************************************************************************
//
// Constants
//
//*****************************************************************************
constexpr uint8_t NUM_REGISTERS = 10;

//*****************************************************************************
//
// Command byte formatting
//
//*****************************************************************************

/* Command byte definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |            C[2:0]           |   MUL   |                A[3:0]                 |
 * ---------------------------------------------------------------------------------
 */

/* SPI Commands */
constexpr uint8_t OPCODE_READ_DIRECT = 0x00;
constexpr uint8_t OPCODE_READ_COMMAND = 0x30; // Includes MUL bit
constexpr uint8_t OPCODE_RREG = 0x40;
constexpr uint8_t OPCODE_WREG = 0x60;
constexpr uint8_t OPCODE_PULSE_CONVERT = 0x80;
constexpr uint8_t OPCODE_RESET = 0xC0;

/* Commands byte masks */
constexpr uint8_t OPCODE_C_MASK = 0xE0;
constexpr uint8_t OPCODE_MUL_MASK = 0x10;
constexpr uint8_t OPCODE_A_MASK = 0x0F;

/* Read mode enum */
typedef enum
{
    DIRECT,
    COMMAND
} readMode;

//*****************************************************************************
//
// Status byte formatting
//
//*****************************************************************************

/* STATUS byte definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   NEW   |   OVF   |  SUPPLY |                    CHID[4:0]                    |
 * ---------------------------------------------------------------------------------
 */

/* STATUS byte field masks */
constexpr uint8_t STATUS_NEW_MASK = 0x80;    /* Indicates new data */
constexpr uint8_t STATUS_OVF_MASK = 0x40;    /* Indicates differential over-range condition */
constexpr uint8_t STATUS_SUPPLY_MASK = 0x20; /* Indicates low analog power-supply condition */
constexpr uint8_t STATUS_CHID_MASK = 0x1F;   /* Channel ID bits */

/* CHID field values */
constexpr uint8_t STATUS_CHID_DIFF0 = 0x00;
constexpr uint8_t STATUS_CHID_DIFF1 = 0x01;
constexpr uint8_t STATUS_CHID_DIFF2 = 0x02;
constexpr uint8_t STATUS_CHID_DIFF3 = 0x03;
constexpr uint8_t STATUS_CHID_DIFF4 = 0x04;
constexpr uint8_t STATUS_CHID_DIFF5 = 0x05;
constexpr uint8_t STATUS_CHID_DIFF6 = 0x06;
constexpr uint8_t STATUS_CHID_DIFF7 = 0x07;
constexpr uint8_t STATUS_CHID_AIN0 = 0x08;
constexpr uint8_t STATUS_CHID_AIN1 = 0x09;
constexpr uint8_t STATUS_CHID_AIN2 = 0x0A;
constexpr uint8_t STATUS_CHID_AIN3 = 0x0B;
constexpr uint8_t STATUS_CHID_AIN4 = 0x0C;
constexpr uint8_t STATUS_CHID_AIN5 = 0x0D;
constexpr uint8_t STATUS_CHID_AIN6 = 0x0E;
constexpr uint8_t STATUS_CHID_AIN7 = 0x0F;
constexpr uint8_t STATUS_CHID_AIN8 = 0x10;
constexpr uint8_t STATUS_CHID_AIN9 = 0x11;
constexpr uint8_t STATUS_CHID_AIN10 = 0x12;
constexpr uint8_t STATUS_CHID_AIN11 = 0x13;
constexpr uint8_t STATUS_CHID_AIN12 = 0x14;
constexpr uint8_t STATUS_CHID_AIN13 = 0x15;
constexpr uint8_t STATUS_CHID_AIN14 = 0x16;
constexpr uint8_t STATUS_CHID_AIN15 = 0x17;
constexpr uint8_t STATUS_CHID_OFFSET = 0x18;
constexpr uint8_t STATUS_CHID_VCC = 0x1A;
constexpr uint8_t STATUS_CHID_TEMP = 0x1B;
constexpr uint8_t STATUS_CHID_GAIN = 0x1C;
constexpr uint8_t STATUS_CHID_REF = 0x1D;
constexpr uint8_t STATUS_CHID_FIXEDCHMODE = 0x1F; /* ID for fixed-channel mode */

//*****************************************************************************
//
// Register definitions
//
//*****************************************************************************

/* Register 0x00 (CONFIG0; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |  SPIRST |  MUXMOD |  BYPAS  |  CLKENB |   CHOP  |   STAT  |    0    |
 * ---------------------------------------------------------------------------------
 */

/** CONFIG0 register address */
constexpr uint8_t REG_ADDR_CONFIG0 = 0x00;

/** CONFIG0 default (reset; value */
constexpr uint8_t CONFIG0_DEFAULT = 0x0A;

/* CONFIG0 register field masks */
constexpr uint8_t CONFIG0_SPIRST_MASK = 0x40;
constexpr uint8_t CONFIG0_MUXMOD_MASK = 0x20;
constexpr uint8_t CONFIG0_BYPAS_MASK = 0x10;
constexpr uint8_t CONFIG0_CLKENB_MASK = 0x08;
constexpr uint8_t CONFIG0_CHOP_MASK = 0x04;
constexpr uint8_t CONFIG0_STAT_MASK = 0x02;

/* Register 0x01 (CONFIG1; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  IDLMOD |           DLY[2:0]          |     SCBCS[1:0]    |     DRATE[0:1]    |
 * ---------------------------------------------------------------------------------
 */

/** CONFIG1 register address */
constexpr uint8_t REG_ADDR_CONFIG1 = 0x01;

/** CONFIG1 default (reset; value */
constexpr uint8_t CONFIG1_DEFAULT = 0x83;

/* CONFIG1 register field masks */
constexpr uint8_t CONFIG1_IDLMOD_MASK = 0x80;
constexpr uint8_t CONFIG1_DLY_MASK = 0x70;
constexpr uint8_t CONFIG1_SCBCS_MASK = 0x0C;
constexpr uint8_t CONFIG1_DRATE_MASK = 0x03;

/* DLY field values */
constexpr uint8_t CONFIG1_DLY_0us = 0x00;
constexpr uint8_t CONFIG1_DLY_8us = 0x10;
constexpr uint8_t CONFIG1_DLY_16us = 0x20;
constexpr uint8_t CONFIG1_DLY_32us = 0x30;
constexpr uint8_t CONFIG1_DLY_64us = 0x40;
constexpr uint8_t CONFIG1_DLY_128us = 0x50;
constexpr uint8_t CONFIG1_DLY_256us = 0x60;
constexpr uint8_t CONFIG1_DLY_384us = 0x70;

/* SCBCS field values */
constexpr uint8_t CONFIG1_SCBCS_OFF = 0x00;
constexpr uint8_t CONFIG1_SCBCS_1_5uA = 0x40;
constexpr uint8_t CONFIG1_SCBCS_24uA = 0xC0;

/* DRATE field values (fixed-channel DRs shown; */
constexpr uint8_t CONFIG1_DRATE_1953SPS = 0x00;
constexpr uint8_t CONFIG1_DRATE_7813SPS = 0x01;
constexpr uint8_t CONFIG1_DRATE_31250SPS = 0x02;
constexpr uint8_t CONFIG1_DRATE_125000SPS = 0x03;

/* Register 0x02 (MUXSCH; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |               AINP[3:0]               |               AINN[3:0]               |
 * ---------------------------------------------------------------------------------
 */

/** MUXSCH register address */
constexpr uint8_t REG_ADDR_MUXSCH = 0x02;

/** MUXSCH default (reset; value */
constexpr uint8_t MUXSCH_DEFAULT = 0x00;

/* MUXSCH register field masks */
constexpr uint8_t MUXSCH_AINP_MASK = 0xF0;
constexpr uint8_t MUXSCH_AINN_MASK = 0x0F;

/* AINP field values */
constexpr uint8_t MUXSCH_AINP_AIN0 = 0x00;
constexpr uint8_t MUXSCH_AINP_AIN1 = 0x10;
constexpr uint8_t MUXSCH_AINP_AIN2 = 0x20;
constexpr uint8_t MUXSCH_AINP_AIN3 = 0x30;
constexpr uint8_t MUXSCH_AINP_AIN4 = 0x40;
constexpr uint8_t MUXSCH_AINP_AIN5 = 0x50;
constexpr uint8_t MUXSCH_AINP_AIN6 = 0x60;
constexpr uint8_t MUXSCH_AINP_AIN7 = 0x70;
constexpr uint8_t MUXSCH_AINP_AIN8 = 0x80;
constexpr uint8_t MUXSCH_AINP_AIN9 = 0x90;
constexpr uint8_t MUXSCH_AINP_AIN10 = 0xA0;
constexpr uint8_t MUXSCH_AINP_AIN11 = 0xB0;
constexpr uint8_t MUXSCH_AINP_AIN12 = 0xC0;
constexpr uint8_t MUXSCH_AINP_AIN13 = 0xD0;
constexpr uint8_t MUXSCH_AINP_AIN14 = 0xE0;
constexpr uint8_t MUXSCH_AINP_AIN15 = 0xF0;

/* AINN field values */
constexpr uint8_t MUXSCH_AINN_AIN0 = 0x00;
constexpr uint8_t MUXSCH_AINN_AIN1 = 0x01;
constexpr uint8_t MUXSCH_AINN_AIN2 = 0x02;
constexpr uint8_t MUXSCH_AINN_AIN3 = 0x03;
constexpr uint8_t MUXSCH_AINN_AIN4 = 0x04;
constexpr uint8_t MUXSCH_AINN_AIN5 = 0x05;
constexpr uint8_t MUXSCH_AINN_AIN6 = 0x06;
constexpr uint8_t MUXSCH_AINN_AIN7 = 0x07;
constexpr uint8_t MUXSCH_AINN_AIN8 = 0x08;
constexpr uint8_t MUXSCH_AINN_AIN9 = 0x09;
constexpr uint8_t MUXSCH_AINN_AIN10 = 0x0A;
constexpr uint8_t MUXSCH_AINN_AIN11 = 0x0B;
constexpr uint8_t MUXSCH_AINN_AIN12 = 0x0C;
constexpr uint8_t MUXSCH_AINN_AIN13 = 0x0D;
constexpr uint8_t MUXSCH_AINN_AIN14 = 0x0E;
constexpr uint8_t MUXSCH_AINN_AIN15 = 0x0F;

/* Register 0x03 (MUXDIF; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  DIFF7  |  DIFF6  |  DIFF5  |  DIFF4  |  DIFF3  |  DIFF2  |  DIFF1  |  DIFF0  |
 * ---------------------------------------------------------------------------------
 */

/** MUXDIF register address */
constexpr uint8_t REG_ADDR_MUXDIF = 0x03;

/** MUXDIF default (reset; value */
constexpr uint8_t MUXDIF_DEFAULT = 0x00;

/* MUXDIF register field masks */
constexpr uint8_t MUXDIF_DIFF7_ENABLE = 0x80;
constexpr uint8_t MUXDIF_DIFF6_ENABLE = 0x40;
constexpr uint8_t MUXDIF_DIFF5_ENABLE = 0x20;
constexpr uint8_t MUXDIF_DIFF4_ENABLE = 0x10;
constexpr uint8_t MUXDIF_DIFF3_ENABLE = 0x08;
constexpr uint8_t MUXDIF_DIFF2_ENABLE = 0x04;
constexpr uint8_t MUXDIF_DIFF1_ENABLE = 0x02;
constexpr uint8_t MUXDIF_DIFF0_ENABLE = 0x01;

/* Register 0x04 (MUXSG0; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   AIN7  |   AIN6  |   AIN5  |   AIN4  |   AIN3  |   AIN2  |   AIN1  |   AIN0  |
 * ---------------------------------------------------------------------------------
 */

/** MUXSG0 register address */
constexpr uint8_t REG_ADDR_MUXSG0 = 0x04;

/** MUXSG0 default (reset; value */
constexpr uint8_t MUXSG0_DEFAULT = 0xFF;

/* MUXSG0 register field masks */
constexpr uint8_t MUXSG0_AIN7_ENABLE = 0x80;
constexpr uint8_t MUXSG0_AIN6_ENABLE = 0x40;
constexpr uint8_t MUXSG0_AIN5_ENABLE = 0x20;
constexpr uint8_t MUXSG0_AIN4_ENABLE = 0x10;
constexpr uint8_t MUXSG0_AIN3_ENABLE = 0x08;
constexpr uint8_t MUXSG0_AIN2_ENABLE = 0x04;
constexpr uint8_t MUXSG0_AIN1_ENABLE = 0x02;
constexpr uint8_t MUXSG0_AIN0_ENABLE = 0x01;

/* Register 0x05 (MUXSG1; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  AIN15  |  AIN14  |  AIN13  |  AIN12  |  AIN11  |  AIN10  |   AIN9  |   AIN8  |
 * ---------------------------------------------------------------------------------
 */

/** MUXSG1 register address */
constexpr uint8_t REG_ADDR_MUXSG1 = 0x05;

/** MUXSG1 default (reset; value */
constexpr uint8_t MUXSG1_DEFAULT = 0xFF;

/* MUXSG1 register field masks */
constexpr uint8_t MUXSG1_AIN15_ENABLE = 0x80;
constexpr uint8_t MUXSG1_AIN14_ENABLE = 0x40;
constexpr uint8_t MUXSG1_AIN13_ENABLE = 0x20;
constexpr uint8_t MUXSG1_AIN12_ENABLE = 0x10;
constexpr uint8_t MUXSG1_AIN11_ENABLE = 0x08;
constexpr uint8_t MUXSG1_AIN10_ENABLE = 0x04;
constexpr uint8_t MUXSG1_AIN9_ENABLE = 0x02;
constexpr uint8_t MUXSG1_AIN8_ENABLE = 0x01;

/* Register 0x06 (SYSRED; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |    0    |   REF   |   GAIN  |   TEMP  |   VCC   |     0   |  OFFSET |
 * ---------------------------------------------------------------------------------
 */

/** SYSRED register address */
constexpr uint8_t REG_ADDR_SYSRED = 0x06;

/** SYSRED default (reset; value */
constexpr uint8_t SYSRED_DEFAULT = 0x00;

/* SYSRED register field masks */
constexpr uint8_t SYSRED_REF_ENABLE = 0x20;
constexpr uint8_t SYSRED_GAIN_ENABLE = 0x10;
constexpr uint8_t SYSRED_TEMP_ENABLE = 0x08;
constexpr uint8_t SYSRED_VCC_ENABLE = 0x04;
constexpr uint8_t SYSRED_OFFSET_ENABLE = 0x01;

/* Register 0x07 (GPIOC; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                    CIO[7:0]                                   |
 * ---------------------------------------------------------------------------------
 */

/** GPIOC register address */
constexpr uint8_t REG_ADDR_GPIOC = 0x07;

/** GPIOC default (reset; value */
constexpr uint8_t GPIOC_DEFAULT = 0xFF;

/* GPIOC register field masks */
constexpr uint8_t GPIOC_GPIO7_INPUT = 0x80;
constexpr uint8_t GPIOC_GPIO6_INPUT = 0x40;
constexpr uint8_t GPIOC_GPIO5_INPUT = 0x20;
constexpr uint8_t GPIOC_GPIO4_INPUT = 0x10;
constexpr uint8_t GPIOC_GPIO3_INPUT = 0x08;
constexpr uint8_t GPIOC_GPIO2_INPUT = 0x04;
constexpr uint8_t GPIOC_GPIO1_INPUT = 0x02;
constexpr uint8_t GPIOC_GPIO0_INPUT = 0x01;

/* Register 0x08 (GPIOD; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                    DIO[7:0]                                   |
 * ---------------------------------------------------------------------------------
 */

/** GPIOD register address */
constexpr uint8_t REG_ADDR_GPIOD = 0x08;

/** GPIOD default (reset; value */
constexpr uint8_t GPIOD_DEFAULT = 0x00;

/* GPIOD register field masks */
constexpr uint8_t GPIOD_GPIO7_HIGH = 0x80;
constexpr uint8_t GPIOD_GPIO6_HIGH = 0x40;
constexpr uint8_t GPIOD_GPIO5_HIGH = 0x20;
constexpr uint8_t GPIOD_GPIO4_HIGH = 0x10;
constexpr uint8_t GPIOD_GPIO3_HIGH = 0x08;
constexpr uint8_t GPIOD_GPIO2_HIGH = 0x04;
constexpr uint8_t GPIOD_GPIO1_HIGH = 0x02;
constexpr uint8_t GPIOD_GPIO0_HIGH = 0x01;

/* Register 0x09 (ID; definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                    ID[7:0]                                    |
 * ---------------------------------------------------------------------------------
 */

/** ID register address */
constexpr uint8_t REG_ADDR_ID = 0x09;

/* ID register field masks */
constexpr uint8_t ID_ID4_MASK = 0x10;

/* ID4 field values */
constexpr uint8_t ID_ID4_ADS1258 = 0x00;
constexpr uint8_t ID_ID4_ADS1158 = 0x10;

#endif /* ADS1258_H_ */
