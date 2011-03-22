/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/
/*
  $Id: at86rf23x_registermap.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
*/

/**
   @addtogroup radio
   @{
   @defgroup radio_registers Radio Registers
   @{

   This is a listing of the radio register definition from the
   AT86RF2xx datasheets.
*/

#ifndef PHY230_REGISTERMAP_EXTERNAL_H
#define PHY230_REGISTERMAP_EXTERNAL_H

/**
   @name Radio Registers
   @{
*/
/** Offset for register TRX_STATUS */
#define RG_TRX_STATUS                    (0x01)
/** Access parameters for sub-register CCA_DONE in register @ref RG_TRX_STATUS */
#define SR_CCA_DONE                  0x01, 0x80, 7
/** Access parameters for sub-register CCA_STATUS in register @ref RG_TRX_STATUS */
#define SR_CCA_STATUS                0x01, 0x40, 6
#define SR_reserved_01_3             0x01, 0x20, 5
/** Access parameters for sub-register TRX_STATUS in register @ref RG_TRX_STATUS */
#define SR_TRX_STATUS                0x01, 0x1f, 0
/** Constant P_ON for sub-register @ref SR_TRX_STATUS */
#define P_ON                     (0)
/** Constant BUSY_RX for sub-register @ref SR_TRX_STATUS */
#define BUSY_RX                  (1)
/** Constant BUSY_TX for sub-register @ref SR_TRX_STATUS */
#define BUSY_TX                  (2)
/** Constant RX_ON for sub-register @ref SR_TRX_STATUS */
#define RX_ON                    (6)
/** Constant TRX_OFF for sub-register @ref SR_TRX_STATUS */
#define TRX_OFF                  (8)
/** Constant PLL_ON for sub-register @ref SR_TRX_STATUS */
#define PLL_ON                   (9)
/** Constant SLEEP for sub-register @ref SR_TRX_STATUS */
#define SLEEP_REG                (15)
/** Constant BUSY_RX_AACK for sub-register @ref SR_TRX_STATUS */
#define BUSY_RX_AACK             (17)
/** Constant BUSY_TX_ARET for sub-register @ref SR_TRX_STATUS */
#define BUSY_TX_ARET             (18)
/** Constant RX_AACK_ON for sub-register @ref SR_TRX_STATUS */
#define RX_AACK_ON               (22)
/** Constant TX_ARET_ON for sub-register @ref SR_TRX_STATUS */
#define TX_ARET_ON               (25)
/** Constant RX_ON_NOCLK for sub-register @ref SR_TRX_STATUS */
#define RX_ON_NOCLK              (28)
/** Constant RX_AACK_ON_NOCLK for sub-register @ref SR_TRX_STATUS */
#define RX_AACK_ON_NOCLK         (29)
/** Constant BUSY_RX_AACK_NOCLK for sub-register @ref SR_TRX_STATUS */
#define BUSY_RX_AACK_NOCLK       (30)
/** Offset for register TRX_STATE */
#define RG_TRX_STATE                     (0x02)
/** Access parameters for sub-register TRAC_STATUS in register @ref RG_TRX_STATE */
#define SR_TRAC_STATUS               0x02, 0xe0, 5
/** Access parameters for sub-register TRX_CMD in register @ref RG_TRX_STATE */
#define SR_TRX_CMD                   0x02, 0x1f, 0
/** Offset for register TRX_CTRL_0 */
#define RG_TRX_CTRL_0                    (0x03)
/** Offset for register TRX_CTRL_1 */
#define RG_TRX_CTRL_1                    (0x04)
/** Access parameters for sub-register IRQ_MASK_MODE in register @ref RG_TRX_CTRL_1 */
#define SR_IRQ_MASK_MODE             0x04, 0x02, 1
/** Access parameters for sub-register PAD_IO in register @ref RG_TRX_CTRL_0 */
#define SR_PAD_IO                    0x03, 0xc0, 6
/** Access parameters for sub-register PAD_IO_CLKM in register @ref RG_TRX_CTRL_0 */
#define SR_PAD_IO_CLKM               0x03, 0x30, 4
/** Access parameters for sub-register CLKM_SHA_SEL in register @ref RG_TRX_CTRL_0 */
#define SR_CLKM_SHA_SEL              0x03, 0x08, 3
/** Access parameters for sub-register CLKM_CTRL in register @ref RG_TRX_CTRL_0 */
#define SR_CLKM_CTRL                 0x03, 0x07, 0
/** Offset for register PHY_TX_PWR */
#define RG_PHY_TX_PWR                    (0x05)
/** Access parameters for sub-register TX_AUTO_CRC_ON in register @ref RG_PHY_TX_PWR.
    Note that the RF230 and RF231 have different register locations for this setting.
    This is the setting for the RF230 part.
 */
#define SR_TX_AUTO_CRC_ON_230            0x05, 0x80, 7
/** Access parameters for sub-register TX_AUTO_CRC_ON in register @ref RG_PHY_TX_PWR.
    Note that the RF230 and RF231 have different register locations for this setting.
    This is the setting for the RF231 part.
 */
#define SR_TX_AUTO_CRC_ON_231            0x04, 0x20, 5

#define SR_reserved_05_2             0x05, 0x70, 4
/** Access parameters for sub-register TX_PWR in register @ref RG_PHY_TX_PWR */
#define SR_TX_PWR                    0x05, 0x0f, 0
/** Offset for register PHY_RSSI */
#define RG_PHY_RSSI                      (0x06)
#define SR_reserved_06_1             0x06, 0xe0, 5
/** Access parameters for sub-register RSSI in register @ref RG_PHY_RSSI */
#define SR_RSSI                      0x06, 0x1f, 0
/** Access parameters for sub-register RND_VALUE in register @ref RG_PHY_RSSI */
#define SR_RND_VALUE                 0x06, 0x60, 5
/** Offset for register PHY_ED_LEVEL */
#define RG_PHY_ED_LEVEL                  (0x07)
/** Offset for register PHY_CC_CCA */
#define RG_PHY_CC_CCA                    (0x08)
/** Access parameters for sub-register CCA_REQUEST in register @ref RG_PHY_CC_CCA */
#define SR_CCA_REQUEST               0x08, 0x80, 7
/** Access parameters for sub-register CCA_MODE in register @ref RG_PHY_CC_CCA */
#define SR_CCA_MODE                  0x08, 0x60, 5
/** Access parameters for sub-register CHANNEL in register @ref RG_PHY_CC_CCA */
#define SR_CHANNEL                   0x08, 0x1f, 0
/** Offset for register CCA_THRES */
#define RG_CCA_THRES                     (0x09)
/** Access parameters for sub-register CCA_CS_THRES in register @ref RG_CCA_THRES */
#define SR_CCA_CS_THRES              0x09, 0xf0, 4
/** Access parameters for sub-register CCA_ED_THRES in register @ref RG_CCA_THRES */
#define SR_CCA_ED_THRES              0x09, 0x0f, 0

/** Offset for register TRX_CTRL_2 */
#define RG_TRX_CTRL_2                    (0x0c)
/** Access parameters for sub-register RX_SAFE_MODE in register @ref RG_TRX_CTRL_2 */
#define SR_RX_SAFE_MODE              0x0c, 0x80, 7

/** Offset for register IRQ_MASK */
#define RG_IRQ_MASK                      (0x0e)
/** Offset for register IRQ_STATUS */
#define RG_IRQ_STATUS                    (0x0f)
/** Offset for register VREG_CTRL */
#define RG_VREG_CTRL                     (0x10)
/** Access parameters for sub-register AVREG_EXT in register @ref RG_VREG_CTRL */
#define SR_AVREG_EXT                 0x10, 0x80, 7
/** Access parameters for sub-register AVDD_OK in register @ref RG_VREG_CTRL */
#define SR_AVDD_OK                   0x10, 0x40, 6
/** Access parameters for sub-register AVREG_TRIM in register @ref RG_VREG_CTRL */
#define SR_AVREG_TRIM                0x10, 0x30, 4
/** Access parameters for sub-register DVREG_EXT in register @ref RG_VREG_CTRL */
#define SR_DVREG_EXT                 0x10, 0x08, 3
/** Access parameters for sub-register DVDD_OK in register @ref RG_VREG_CTRL */
#define SR_DVDD_OK                   0x10, 0x04, 2
/** Access parameters for sub-register DVREG_TRIM in register @ref RG_VREG_CTRL */
#define SR_DVREG_TRIM                0x10, 0x03, 0
/** Offset for register BATMON */
#define RG_BATMON                        (0x11)
#define SR_reserved_11_1             0x11, 0xc0, 6
/** Access parameters for sub-register BATMON_OK in register @ref RG_BATMON */
#define SR_BATMON_OK                 0x11, 0x20, 5
/** Access parameters for sub-register BATMON_HR in register @ref RG_BATMON */
#define SR_BATMON_HR                 0x11, 0x10, 4
/** Access parameters for sub-register BATMON_VTH in register @ref RG_BATMON */
#define SR_BATMON_VTH                0x11, 0x0f, 0
/** Offset for register XOSC_CTRL */
#define RG_XOSC_CTRL                     (0x12)
/** Offset for register RX_SYN */
#define RG_RX_SYN                        0x15
/** Offset for sub-register RX_PDT_LEVEL */
#define SR_RX_PDT_LEVEL              0x15, 0x0f, 0
/** Offset for register XAH_CTRL_1 */
#define RG_XAH_CTRL_1                      0x17
/** Access parameters for sub-register XTAL_MODE in register @ref RG_XOSC_CTRL */
#define SR_XTAL_MODE                 0x12, 0xf0, 4
/** Access parameters for sub-register XTAL_TRIM in register @ref RG_XOSC_CTRL */
#define SR_XTAL_TRIM                 0x12, 0x0f, 0

#define SR_AACK_ACK_TIME             0x17, 0x04, 2

/** Offset for register FTN_CTRL */
#define RG_FTN_CTRL                      (0x18)
/** Access parameters for sub-register FTN_START in register @ref RG_FTN_CTRL */
#define SR_FTN_START                 0x18, 0x80, 7
#define SR_reserved_18_2             0x18, 0x40, 6
/** Access parameters for sub-register FTNV in register @ref RG_FTN_CTRL */
#define SR_FTNV                      0x18, 0x3f, 0
/** Offset for register PLL_CF */
#define RG_PLL_CF                        (0x1a)
/** Access parameters for sub-register PLL_CF_START in register @ref RG_PLL_CF */
#define SR_PLL_CF_START              0x1a, 0x80, 7
#define SR_reserved_1a_2             0x1a, 0x70, 4
/** Access parameters for sub-register PLL_CF in register @ref RG_PLL_CF */
#define SR_PLL_CF                    0x1a, 0x0f, 0
/** Offset for register PLL_DCU */
#define RG_PLL_DCU                       (0x1b)
/** Access parameters for sub-register PLL_DCU_START in register @ref RG_PLL_DCU */
#define SR_PLL_DCU_START             0x1b, 0x80, 7
#define SR_reserved_1b_2             0x1b, 0x40, 6
/** Access parameters for sub-register PLL_DCUW in register @ref RG_PLL_DCU */
#define SR_PLL_DCUW                  0x1b, 0x3f, 0
/** Offset for register PART_NUM */
#define RG_PART_NUM                      (0x1c)
/** Offset for register VERSION_NUM */
#define RG_VERSION_NUM                   (0x1d)
/** Offset for register MAN_ID_0 */
#define RG_MAN_ID_0                      (0x1e)
/** Offset for register MAN_ID_1 */
#define RG_MAN_ID_1                      (0x1f)
/** Offset for register XAH_CTRL */
#define RG_XAH_CTRL_0                     (0x2c)
/** Access parameters for sub-register MAX_FRAME_RETRIES in register @ref RG_XAH_CTRL_0 */
#define SR_MAX_FRAME_RETRIES         0x2c, 0xf0, 4
/** Access parameters for sub-register MAX_CSMA_RETRIES in register @ref RG_XAH_CTRL_0 */
#define SR_MAX_CSMA_RETRIES          0x2c, 0x0e, 1
#define SR_reserved_2c_3             0x2c, 0x01, 0
/** Offset for register CSMA_BE */
#define RG_CSMA_BE                      0x2f
/** Access parameters for sub-register MIN_BE in register @ref RG_CSMA_SEED_1 */
#define SR_MIN_BE                    0x2e, 0xc0, 6
#define SR_reserved_2e_2             0x2e, 0x30, 4
/** Access parameters for sub-register I_AM_COORD in register @ref RG_CSMA_SEED_1 */
#define SR_I_AM_COORD                0x2e, 0x08, 3
/** @} */

/**
   @name Radio commands.
   See @ref SR_TRX_CMD and radio datasheet for details.
   @{
*/
/** Constant CMD_NOP for sub-register @ref SR_TRX_CMD */
#define CMD_NOP                  (0)
/** Constant CMD_TX_START for sub-register @ref SR_TRX_CMD */
#define CMD_TX_START             (2)
/** Constant CMD_FORCE_TRX_OFF for sub-register @ref SR_TRX_CMD */
#define CMD_FORCE_TRX_OFF        (3)
/** Constant CMD_RX_ON for sub-register @ref SR_TRX_CMD */
#define CMD_RX_ON                (6)
/** Constant CMD_TRX_OFF for sub-register @ref SR_TRX_CMD */
#define CMD_TRX_OFF              (8)
/** Constant CMD_PLL_ON for sub-register @ref SR_TRX_CMD */
#define CMD_PLL_ON               (9)
/** Constant CMD_RX_AACK_ON for sub-register @ref SR_TRX_CMD */
#define CMD_RX_AACK_ON           (22)
/** Constant CMD_TX_ARET_ON for sub-register @ref SR_TRX_CMD */
#define CMD_TX_ARET_ON           (25)
/** @} */
/**
   @name Constants for CLKM current.
   See @ref SR_PAD_IO_CLKM.
   @{
*/
/** Constant CLKM_2mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_2mA                 (0)
/** Constant CLKM_4mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_4mA                 (1)
/** Constant CLKM_6mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_6mA                 (2)
/** Constant CLKM_8mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_8mA                 (3)
/** @} */
/**
   @name Constants that define the CLKM frequency.

   See @ref SR_CLKM_CTRL and radio datasheet.
   @{
*/
/** Constant CLKM_no_clock for sub-register @ref SR_CLKM_CTRL */
#define CLKM_no_clock            (0)
/** Constant CLKM_1MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_1MHz                (1)
/** Constant CLKM_2MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_2MHz                (2)
/** Constant CLKM_4MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_4MHz                (3)
/** Constant CLKM_8MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_8MHz                (4)
/** Constant CLKM_16MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_16MHz               (5)
/** @} */
/**
   @name Constants that define the bits in IRQ_STATUS register.
   @{
*/
/** Access parameters for sub-register IRQ_7_BAT_LOW in register @ref RG_IRQ_STATUS */
#define SR_IRQ_7_BAT_LOW             0x0f, 0x80, 7
/** Access parameters for sub-register IRQ_6_TRX_UR in register @ref RG_IRQ_STATUS */
#define SR_IRQ_6_TRX_UR              0x0f, 0x40, 6
/** Access parameters for sub-register IRQ_5 in register @ref RG_IRQ_STATUS */
#define SR_IRQ_5                     0x0f, 0x20, 5
/** Access parameters for sub-register IRQ_4 in register @ref RG_IRQ_STATUS */
#define SR_IRQ_4                     0x0f, 0x10, 4
/** Access parameters for sub-register IRQ_3_TRX_END in register @ref RG_IRQ_STATUS */
#define SR_IRQ_3_TRX_END             0x0f, 0x08, 3
/** Access parameters for sub-register IRQ_2_RX_START in register @ref RG_IRQ_STATUS */
#define SR_IRQ_2_RX_START            0x0f, 0x04, 2
/** Access parameters for sub-register IRQ_1_PLL_UNLOCK in register @ref RG_IRQ_STATUS */
#define SR_IRQ_1_PLL_UNLOCK          0x0f, 0x02, 1
/** Access parameters for sub-register IRQ_0_PLL_LOCK in register @ref RG_IRQ_STATUS */
#define SR_IRQ_0_PLL_LOCK            0x0f, 0x01, 0
/** @} */
/**
   @name Constants that define the values in the AVREG_TRIM sub-register.
   @{
*/
/** Constant AVREG_1_80V for sub-register @ref SR_AVREG_TRIM */
#define AVREG_1_80V              (0)
/** Constant AVREG_1_75V for sub-register @ref SR_AVREG_TRIM */
#define AVREG_1_75V              (1)
/** Constant AVREG_1_84V for sub-register @ref SR_AVREG_TRIM */
#define AVREG_1_84V              (2)
/** Constant AVREG_1_88V for sub-register @ref SR_AVREG_TRIM */
#define AVREG_1_88V              (3)
/** @} */
/**
   @name Constants that define the values for the DVREG_TRIM sub-register.
   @{
*/
/** Constant DVREG_1_80V for sub-register @ref SR_DVREG_TRIM */
#define DVREG_1_80V              (0)
/** Constant DVREG_1_75V for sub-register @ref SR_DVREG_TRIM */
#define DVREG_1_75V              (1)
/** Constant DVREG_1_84V for sub-register @ref SR_DVREG_TRIM */
#define DVREG_1_84V              (2)
/** Constant DVREG_1_88V for sub-register @ref SR_DVREG_TRIM */
#define DVREG_1_88V              (3)
/** @} */
/**
    @name Constants that define the different part numbers.

    See @ref RG_PART_NUM and radio chip datasheet for correct values.

    @{
*/
#define RF230                    (2)   ///< Value for AT86RF230
#define RF231                    (3)   ///< Value for AT86RF231
#define RF212                    (7)   ///< Value for AT86RF212
/** @} */
/**
   @name Registers that hold the node's short address in the radio.
   @{
*/
/** Offset for register SHORT_ADDR_0 */
#define RG_SHORT_ADDR_0                  (0x20)
/** Offset for register SHORT_ADDR_1 */
#define RG_SHORT_ADDR_1                  (0x21)
/** @} */
/**
   @name Registers that hold the node's PAN ID in the radio.
   @{
*/
/** Offset for register PAN_ID_0 */
#define RG_PAN_ID_0                      (0x22)
/** Offset for register PAN_ID_1 */
#define RG_PAN_ID_1                      (0x23)
/** @} */
/**
   @name Registers that hold the node's MAC Address in the radio.
   @{
*/
/** Offset for register IEEE_ADDR_0 */
#define RG_IEEE_ADDR_0                   (0x24)
/** Offset for register IEEE_ADDR_1 */
#define RG_IEEE_ADDR_1                   (0x25)
/** Offset for register IEEE_ADDR_2 */
#define RG_IEEE_ADDR_2                   (0x26)
/** Offset for register IEEE_ADDR_3 */
#define RG_IEEE_ADDR_3                   (0x27)
/** Offset for register IEEE_ADDR_4 */
#define RG_IEEE_ADDR_4                   (0x28)
/** Offset for register IEEE_ADDR_5 */
#define RG_IEEE_ADDR_5                   (0x29)
/** Offset for register IEEE_ADDR_6 */
#define RG_IEEE_ADDR_6                   (0x2a)
/** Offset for register IEEE_ADDR_7 */
#define RG_IEEE_ADDR_7                   (0x2b)
/** @} */
/**
   @name Registers that hold the node's CSMA seed value.
   @{
*/
/** Offset for register CSMA_SEED_0 */
#define RG_CSMA_SEED_0                   (0x2d)
/** Offset for register CSMA_SEED_1 */
#define RG_CSMA_SEED_1                   (0x2e)
/** @} */
/** @name Channel range definitions

    These two macros define the lowest and highest channel for a given
    band.  Possible channels are 0-10 (900MHz band) and 11-26 (2.4GHz
    band).
    @{
 */
#define MIN_CHANNEL  (11)
#define MAX_CHANNEL  (26)
/** @} */

// Dummy defines to make code compile
/** @name Definition of the modulation parameters for the RF212 chip.
    These parameters get written to RG_TRX_CTRL_2 to set the
    modulation mode.  Note that many more modes are possible, but only
    the modes allowed by IEEE 802.15.4 are shown here.  See the RF212
    datasheet for the other modes.  @ingroup radio_registers @{
*/
#define BPSK_20         0x20     ///< 20 Kbps
#define BPSK_40         0x24     ///< 40 Kbps
#define OQPSK_100       0x08     ///< 100 Kbps
#define OQPSK_SIN_250   0x2c     ///< 250 Kbps, half-sine filtering
#define OQPSK_RC_250    0x1c     ///< 250 Kbps, RC filtering
/** @} */
#define SR_GC_TX_OFFS                0x16, 0x03, 0
#define RG_RF_CTRL_0                 (0x16)
#define SR_AACK_UPLD_RES_FT          0x17, 0x10, 4
#define SR_AACK_FLTR_RES_FT          0x17, 0x20, 5
#define SR_CC_BAND                   0x14, 0x07, 0
#define SR_TX_AUTO_CRC_ON            0x04, 0x20, 5
/**
   @name These registers control the radio's carrier frequency (RF212 only).
   @{
*/
#define RG_CC_CTRL_0                      (0x13)  ///< Channel control register 0
#define RG_CC_CTRL_1                      (0x14)  ///< Channel control register 1
/** @} */

/** @} */ // End of radio_registers
/** @} */ // End of addtogroup radio

#endif /* PHY230_REGISTERMAP_EXTERNAL_H */
