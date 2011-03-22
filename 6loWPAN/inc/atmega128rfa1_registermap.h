/*
 * atmega128rfa1_registermap.h
 *
 *  Created on: 10.03.2010
 *      Author: ele
 */

#ifndef ATMEGA128RFA1_REGISTERMAP_H_
#define ATMEGA128RFA1_REGISTERMAP_H_

#define RG_IRQ_MASK                       (0x14E)
#define RG_PHY_ED_LEVEL                   (0x147)
#define RG_PAN_ID_0                       (0x162)
#define RG_PAN_ID_1                       (0x163)
#define RG_SHORT_ADDR_0                   (0x160)
#define RG_SHORT_ADDR_1                   (0x161)
#define RG_IEEE_ADDR_0                    (0x164)
#define RG_IEEE_ADDR_1                    (0x165)
#define RG_IEEE_ADDR_2                    (0x166)
#define RG_IEEE_ADDR_3                    (0x167)
#define RG_IEEE_ADDR_4                    (0x168)
#define RG_IEEE_ADDR_5                    (0x169)
#define RG_IEEE_ADDR_6                    (0x16A)
#define RG_IEEE_ADDR_7                    (0x16B)

#define RG_TRX_CTRL_2                     (0x14C)
#define RG_PHY_TX_PWR                     (0x145)
#define RG_CSMA_BE                        (0x16F)
#define RG_TST_RX_LENGTH                  (0x17B)
#define RG_IRQ_STATUS                     (0x14F)
#define RG_PART_NUM                       (0x15C)
#define RG_VERSION_NUM                    (0x15D)
#define RG_BATMON                         (0x151)

/**
    @name Constants that define the different part numbers.
    See @ref RG_PART_NUM and radio chip datasheet for correct values.
    @{
*/
#define RF230                             (2)     ///< Value for AT86RF230
#define RF231                             (3)     ///< Value for AT86RF231
#define RF212                             (7)     ///< Value for AT86RF212
#define RF128RFA1                         (0x83)  ///< Value for ATmega128RFA1 // added by de

/** @} */
//#define HAL_BAT_LOW_MASK
//#define HAL_ED_READY_MASK
//#define RF2xx_SUPPORTED_INTERRUPT_MASK


#define SR_AACK_ACK_TIME               0x157, 0x04, 2
#define SR_MAX_FRAME_RETRIES           0x16C, 0xf0, 4
#define SR_RX_PDT_LEVEL                0x155, 0x0f, 0
#define SR_TRX_CMD                     0x142, 0x1f, 0
#define SR_CLKM_SHA_SEL                // there is no clock source selection on 128rfa1

/* Auto CRC calculation - all values have to be set - that keeps compiler happy */
#define SR_TX_AUTO_CRC_ON_230          0x05,  0x80, 7 //values from RF230
#define SR_TX_AUTO_CRC_ON_231          0x04,  0x20, 5 //values from RF212
#define SR_TX_AUTO_CRC_ON              0x144, 0x20, 5

#define SR_I_AM_COORD                  0x16E, 0x08, 3
#define SR_GC_TX_OFFS                  0x16,  0x03, 0

#define SR_RSSI                        0x146, 0x1f, 0
#define SR_TRAC_STATUS                 0x142, 0xe0, 5
#define SR_CHANNEL                     0x148, 0x1f, 0
#define SR_TX_PWR                      0x145, 0x0f, 0
#define SR_CCA_MODE                    0x148, 0x60, 5
#define SR_CCA_ED_THRES                0x149, 0x0f, 0

#define SR_BATMON_VTH                  0x151, 0x0f, 0
#define SR_BATMON_HR                   0x151, 0x10, 4
#define SR_BATMON_OK                   0x151, 0x20, 5

#define SR_CLKM_CTRL                   // does not exist -> there is no radio clock source control on 128rfa1

#define SR_TRX_STATUS                  0x141, 0x1f, 0
#define SR_RND_VALUE                   0x146, 0x60, 5 // 2bit random value

/** @} */
/** @name Channel range definitions

    These two macros define the lowest and highest channel for a given
    band.  Possible channels are 0-10 (900MHz band) and 11-26 (2.4GHz
    band).
    @{
 */
#define MIN_CHANNEL                    (11)
#define MAX_CHANNEL                    (26)
/** @} */




// Dummy defines to make code compile

/** @name Definition of the modulation parameters for the RF212 chip.
 *  @{
*/
#define BPSK_20                        0x20     ///< 20 Kbps
#define BPSK_40                        0x24     ///< 40 Kbps
#define OQPSK_100                      0x08     ///< 100 Kbps
#define OQPSK_SIN_250                  0x2c     ///< 250 Kbps, half-sine filtering
#define OQPSK_RC_250                   0x1c     ///< 250 Kbps, RC filtering

#define RG_RF_CTRL_0                   (0x16)
#define RG_CC_CTRL_0                   (0x13)  ///< Channel control register 0
#define RG_CC_CTRL_1                   (0x14)  ///< Channel control register 1
#define SR_AACK_UPLD_RES_FT            0x17, 0x10, 4
#define SR_AACK_FLTR_RES_FT            0x17, 0x20, 5
#define SR_CC_BAND                     0x14, 0x07, 0
#define SR_GC_TX_OFFS                  0x16, 0x03, 0
#define SR_CC_BAND                     0x14, 0x07, 0
/** @} */


/** does not exist for single chip, but is used in source code, so its just mapped to another status */
#define BUSY_RX_AACK_NOCLK             BUSY_RX_AACK

#endif /* ATMEGA128RFA1_REGISTERMAP_H_ */
