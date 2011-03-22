/*
 * Copyright (c) 2008-2009, Atmel Corporation.
 * All rights reserved.
 *
 * Portions of this file are based off code from the Contiki Operating System,
 * which is copyright Adam Dunkels
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 * $Id: avr_sixlowpan.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
 */

/**
 * @addtogroup mac
 * @{
 * @addtogroup avr6lowpan
 * @{
 *
 * This implements a 6LoWPAN Layer on the AVR Devices. The 6LoWPAN Layer
 * implements the 'draft-hui-6lowpan-hc-01' description of the
 * 6LoWPAN layer, with the following restrictions:
 *
 * - No fragmentation support, hence maximum IP size is restricted to
 *   maximum payload the RUM network can send
 *
 * - The destination IP addresses must always be compressed. On the current
 *   setup the node will ALWAYS have a compressible address.
 *
 * - The nodes detect the 'context' for compression from the first router
 *   advertisement they receive addressed to them.
 *
 * Essentially the first point is the only one you as a user need to worry
 * about. That is that any single IP packet must be limited in size. Since
 * the application can control this, there should never be a need to send
 * a larger packet to such a tiny end-node.
 *
 * The code also implements an IPv6 layer in combination with the 6LoWPAN
 * layer. To keep code size as small as possible, the layers are tightly
 * integrated. The IPv6 layer has the ability to:
 *
 *  - Respond to ping requests
 *
 *  - Generate ping requests
 *
 *  - Generate router solicitations to find the router
 *
 *  - Process router advertisements to autoconfigure it's IPv6 address
 *
 *  - Receive and send UDP data over any valid UDP port
 *
 * The IP layer though has the following restrictions:
 *
 *  - The checksums in incoming IPv6 packets (UDP, ICMP, etc) are never
 *    checked. It is assumed the checksums at the MAC layer were sufficent.
 *    The code does generate valid IPv6 checksums when sending packets out.
 *
 *  - Makes no use of timers in IPv6 messages - aka the prefix is never
 *    expired.
 *
 * It should be noted that the IPv6 layer on the AVR makes no attempt to
 * implement all features of IPv6, or be fully IPv6 complient. This is
 * done in order to make the code as small as possible, while still
 * maintaining the required features to provide connectivity.
 *
 * An example application is provided that demonstrates all the available
 * IPv6 features. This includes generating & responding to pings, sending
 * data autonomously, and interactively allowing a user/server to configure
 * the node and retrieve readings.
 *
 * The code size is around 3KB of FLASH and around 150 bytes of SRAM for the
 * 6LoWPAN / IPv6 code.
 *
 */



#ifndef __AVR_SIXLOWPAN_H__
#define __AVR_SIXLOWPAN_H__
#include "../inc/mac.h"
#include <stdint.h>

/**********************************************************
 **     Typedef's                                        **
 **********************************************************/

typedef uint8_t  u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;


/**********************************************************
 **     Defines                                          **
 **********************************************************/

#define SENSOR_RANDOM_IPSO  10


/**********************************************************
 **     IPv6 Next Header Types                           **
 **********************************************************/

#define UIP_PROTO_ICMP  1
#define UIP_PROTO_TCP   6
#define UIP_PROTO_UDP   17
#define UIP_PROTO_ICMP6 58

/**********************************************************
 **     Size of various headers                          **
 **********************************************************/

#define UIP_IPH_LEN    40
#define UIP_FRAGH_LEN  8

#define UIP_UDPH_LEN    8    /* Size of UDP header */
#define UIP_TCPH_LEN   20    /* Size of TCP header */

#define UIP_ICMPH_LEN   4    /* Size of ICMP header */

#define UIP_IPUDPH_LEN (UIP_UDPH_LEN + UIP_IPH_LEN)    /* Size of IP +
                        * UDP
                               * header */
#define UIP_IPTCPH_LEN (UIP_TCPH_LEN + UIP_IPH_LEN)    /* Size of IP +
                               * TCP
                               * header */
#define UIP_TCPIP_HLEN UIP_IPTCPH_LEN
#define UIP_IPICMPH_LEN (UIP_IPH_LEN + UIP_ICMPH_LEN) /* size of ICMP
                                                         + IP header */


/**********************************************************
 **     ND6 Defines & Types                              **
 **********************************************************/

/** \name ND6 option types */
/** @{ */
#define UIP_ND6_OPT_SLLAO               1
#define UIP_ND6_OPT_TLLAO               2
#define UIP_ND6_OPT_PREFIX_INFO         3
#define UIP_ND6_OPT_REDIRECTED_HDR      4
#define UIP_ND6_OPT_MTU                 5
/** @} */


/** \name ND6 message length (excluding options) */
/** @{ */
#define UIP_ND6_NA_LEN                  20
#define UIP_ND6_NS_LEN                  20
#define UIP_ND6_RA_LEN                  12
#define UIP_ND6_RS_LEN                  4
/** @} */


/** \name ND6 option length in bytes */
/** @{ */
#define UIP_ND6_OPT_HDR_LEN            2
#define UIP_ND6_OPT_PREFIX_INFO_LEN    32
#define UIP_ND6_OPT_MTU_LEN            8

/** \name Neighbor Advertisement flags masks */
/** @{ */
#define UIP_ND6_NA_FLAG_ROUTER          0x80
#define UIP_ND6_NA_FLAG_SOLICITED       0x40
#define UIP_ND6_NA_FLAG_OVERRIDE        0x20
/** @} */

/** \brief ND option: both TLLAO and SLLAO */
struct uip_nd6_opt_llao {
  u8_t type;
  u8_t len;
  u8_t addr[6];
}__attribute__((__packed__));

/**
 * \brief A router solicitation  constant part
 *
 * Possible option is: SLLAO
 */
struct uip_nd6_rs {
  u32_t reserved;
};


/**
 * \brief A neighbor advertisement constant part.
 *
 * Possible option is: TLLAO
 */
struct uip_nd6_na {
  u8_t flagsreserved;
  u8_t reserved[3];
  u8_t tgtipaddr[16];
}__attribute__((__packed__));

/** \brief ND option header */
struct uip_nd6_opt_hdr {
  u8_t type;
  u8_t len;
}__attribute__((__packed__));

/** \brief ND option prefix information */
struct uip_nd6_opt_prefix_info {
  u8_t type;
  u8_t len;
  u8_t preflen;
  u8_t flagsreserved1;
  u32_t validlt;
  u32_t preferredlt;
  u32_t reserved2;
  u8_t prefix[16];
}__attribute__((__packed__));

/**********************************************************
 **     ICMPv6                                           **
 **********************************************************/

/** \name ICMPv6 message types */
/** @{ */
#define ICMP6_DST_UNREACH                 1 /**< dest unreachable */
#define ICMP6_PACKET_TOO_BIG                2   /**< packet too big */
#define ICMP6_TIME_EXCEEDED             3   /**< time exceeded */
#define ICMP6_PARAM_PROB                   4    /**< ip6 header bad */
#define ICMP6_ECHO_REQUEST              128  /**< Echo request */
#define ICMP6_ECHO_REPLY                129  /**< Echo reply */

#define ICMP6_RS                        133  /**< Router Solicitation */
#define ICMP6_RA                        134  /**< Router Advertisement */
#define ICMP6_NS                        135  /**< Neighbor Solicitation */
#define ICMP6_NA                        136  /**< Neighbor advertisement */
#define ICMP6_REDIRECT                  137  /**< Redirect */
/** @} */

/* The ICMP headers. */
struct uip_icmp_hdr {
  u8_t type, icode;
  u16_t icmpchksum;
}__attribute__((packed));

/**********************************************************
 **     Sixlowpan defines, typedefs & structures         **
 **********************************************************/

/**
 * \name General sixlowpan defines
 * @{
 */
/* Min and Max compressable UDP ports */
#define SIXLOWPAN_UDP_PORT_MIN                     0xF0B0
#define SIXLOWPAN_UDP_PORT_MAX                     0xF0BF   /* F0B0 + 15 */
/** @} */


/**
 * \name 6lowpan dispatchs
 * @{
 */
#define SIXLOWPAN_DISPATCH_IPV6                    0x41 /* 01000001 = 65 */
#define SIXLOWPAN_DISPATCH_HC1                     0x42 /* 01000010 = 66 */
#define SIXLOWPAN_DISPATCH_IPHC                    0x03 /* 00000011 = 3 */
#define SIXLOWPAN_DISPATCH_FRAG1                   0xc0 /* 11000xxx */
#define SIXLOWPAN_DISPATCH_FRAGN                   0xe0 /* 11100xxx */
/** @} */

/** \name HC1 encoding
 * @{
 */
#define SIXLOWPAN_HC1_NH_UDP                       0x02
#define SIXLOWPAN_HC1_NH_TCP                       0x06
#define SIXLOWPAN_HC1_NH_ICMP6                     0x04
/** @} */


/* Min and Max compressable UDP ports */
#define SIXLOWPAN_UDP_PORT_MIN                     0xF0B0
#define SIXLOWPAN_UDP_PORT_MAX                     0xF0BF   /* F0B0 + 15 */

/** \name HC_UDP encoding (works together with HC1)
 * @{
 */
#define SIXLOWPAN_HC_UDP_ALL_C                     0xE0
/** @} */

/**
 * \name IPHC encoding
 * @{
 */
/*
 * Values of fields within the IPHC encoding first byte
 * (C stands for compressed and I for inline)
 */
#define SIXLOWPAN_IPHC_TC_C                        0x80
#define SIXLOWPAN_IPHC_VF_C                        0x40
#define SIXLOWPAN_IPHC_NH_C                        0x20
#define SIXLOWPAN_IPHC_TTL_1                       0x08
#define SIXLOWPAN_IPHC_TTL_64                      0x10
#define SIXLOWPAN_IPHC_TTL_255                     0x18
#define SIXLOWPAN_IPHC_TTL_I                       0x00

/* Values of fields within the IPHC encoding second byte */
#define SIXLOWPAN_IPHC_SAM_I                       0x00
#define SIXLOWPAN_IPHC_SAM_64                      0x40
#define SIXLOWPAN_IPHC_SAM_16                      0x80
#define SIXLOWPAN_IPHC_SAM_0                       0xC0
#define SIXLOWPAN_IPHC_DAM_I                       0x00
#define SIXLOWPAN_IPHC_DAM_64                      0x04
#define SIXLOWPAN_IPHC_DAM_16                      0x08
#define SIXLOWPAN_IPHC_DAM_0                       0x0C

/* Link local context number */
#define SIXLOWPAN_IPHC_ADDR_CONTEXT_LL             0
/* 16-bit multicast addresses compression */
#define SIXLOWPAN_IPHC_MCAST_RANGE                 0xA0
/** @} */

/**
 * \name LOWPAN_UDP encoding (works together with IPHC)
 * @{
 */
#define SIXLOWPAN_NHC_UDP_ID                       0xF8
#define SIXLOWPAN_NHC_UDP_C                        0xFB
#define SIXLOWPAN_NHC_UDP_I                        0xF8
/** @} */


/**
 * \name The 6lowpan "headers" length
 * @{
 */

#define SIXLOWPAN_IPV6_HDR_LEN                     1    /*one byte*/
#define SIXLOWPAN_HC1_HDR_LEN                      3
#define SIXLOWPAN_HC1_HC_UDP_HDR_LEN               7
#define SIXLOWPAN_FRAG1_HDR_LEN                    4
#define SIXLOWPAN_FRAGN_HDR_LEN                    5
/** @} */

/**
 * \brief The header for fragments
 * \note We do not define different structuresfor FRAG1
 * and FRAGN headers, which are different. For FRAG1, the
 * offset field is just not used
 */
struct sixlowpan_frag_hdr {
  u16_t dispatch_size;
  u16_t tag;
  u8_t offset;
};

/**
 * \brief The HC1 header when HC_UDP is not used
 *
 * When all fields are compressed and HC_UDP is not used,
 * we use this structure. If HC_UDP is used, the ttl is
 * in another spot, and we use the sixlowpan_hc1_hc_udp
 * structure
 */
struct sixlowpan_hc1_hdr {
  u8_t dispatch;
  u8_t encoding;
  u8_t ttl;
};

/**
 * \brief HC1 followed by HC_UDP
 */
struct sixlowpan_hc1_hc_udp_hdr {
  u8_t dispatch;
  u8_t hc1_encoding;
  u8_t hc_udp_encoding;
  u8_t ttl;
  u8_t ports;
  u16_t udpchksum;
};

/**
 * \brief IPHC dispatch and encoding
 * the rest (uncompressed fields) is variable
 */
struct sixlowpan_iphc_hdr {
  u8_t dispatch;
  u8_t encoding[2];
};

struct sixlowpan_nhc_udp_comp_hdr {
  u8_t nhcid;
  u8_t ports;
  u16_t udpchksum;
};

/**
 * \brief An address context for IPHC address compression
 */
struct sixlowpan_addr_context {
  u8_t used;
  u8_t number;
  u8_t prefix[8];
};

/**
 * \name Address compressibility test functions
 * @{
 */

/**
 * \brief check whether we can compress the IID in
 * address 'a' to 16 bits.
 * This is used for unicast addresses only, and is true
 * if first 49 bits of IID are 0
 */
#define sixlowpan_is_iid_16_bit_compressable(a) \
  ((((a)->u16[4]) == 0) &&                       \
   (((a)->u16[5]) == 0) &&                       \
   (((a)->u16[6]) == 0) &&                       \
   ((((a)->u8[14]) & 0x80) == 0))

/**
 * \brief check whether the 9-bit group-id of the
 * compressed multicast address is known. It is true
 * if the 9-bit group is the all nodes or all routers
 * group.
 * \param a is typed u8_t *
 */
#define sixlowpan_is_mcast_addr_decompressable(a) \
   (((*a & 0x01) == 0) &&                           \
    ((*(a + 1) == 0x01) || (*(a + 1) == 0x02)))

/**
 * \brief check whether the 112-bit group-id of the
 * multicast address is mapable to a 9-bit group-id
 * It is true if the group is the all nodes or all
 * routers group.
*/
#define sixlowpan_is_mcast_addr_compressable(a) \
  ((((a)->u16[1]) == 0) &&                       \
   (((a)->u16[2]) == 0) &&                       \
   (((a)->u16[3]) == 0) &&                       \
   (((a)->u16[4]) == 0) &&                       \
   (((a)->u16[5]) == 0) &&                       \
   (((a)->u16[6]) == 0) &&                       \
   (((a)->u8[14]) == 0) &&                       \
   ((((a)->u8[15]) == 1) || (((a)->u8[15]) == 2)))

/** @} */

#ifndef MSB
#define MSB(x)   ((uint8_t *)&x)[1]
#define LSB(x)   ((uint8_t *)&x)[0]
#endif


/**********************************************************
 **     avr-sixlowpan specific typedefs                  **
 **********************************************************/

typedef struct {
    uint8_t   ttl;
    uint8_t * ttlptr;
    uint8_t   proto;
    uint8_t   destcontext;
    uint8_t   destmode;
    uint8_t   srccontext;
    uint8_t   srcmode;
    uint8_t * srcptr;
    uint8_t   mac_hdr_len;
} ipbuf_t;

/* We just store two things - the prefix checksum, and if the prefix
 * is being used. Could even just store checksum, and reserve 0xFFFF value
 * as indicator prefix is not used! */
typedef struct {
    uint16_t checksum;
    uint8_t  is_used;
} sixlowpan_prefix_t;


/**********************************************************
 **     Function Prototypes                              **
 **********************************************************/

/* General IP functions */
void sixlowpan_init(void);
uint16_t sixlowpan_hc01_pseudochksum(void);

/* ND Functions */
void sixlowpan_hc01_gen_na(ipbuf_t * ipbuf, uint8_t solicited);
void sixlowpan_hc01_gen_rs(void);
void sixlowpan_hc01_process_ra(ipbuf_t * ipbuf);


//void sixlowpan_hc01_process(void);
void sixlowpan_hc01_process(ftData *frame, uint8_t payloadlen);
void sixlowpan_DataIndication(ftData *frame, uint8_t payloadlen);


/* UDP Functions */
uint8_t * sixlowpan_hc01_udp_setup_ipglobal(void);
void sixlowpan_hc01_udp_setup_iplocal(uint16_t addr);
void sixlowpan_hc01_udp_setup_ports(uint16_t srcport, uint16_t destport);
void sixlowpan_hc01_udp_send(void);
uint8_t * sixlowpan_hc01_udp_get_payloadptr(void);
void sixlowpan_hc01_udp_set_payloadsize(uint8_t size);
void sixlowpan_hc01_udp_send(void);

/* Ping Functions */
uint8_t * sixlowpan_hc01_ping_setup_ipglobal(uint8_t sequence);
void sixlowpan_hc01_ping_send(void);



/**
 * \brief User callback when an ICMP Echo Response is received
 * \param sequence The sequence number in the ICMP Echo Response
 *
 * This function allows the user to know if a ping response was
 * successfully received.
 *
 */
void sixlowpan_ping_usercall(uint8_t sequence);

/**
 * \brief User callback when a UDP packet is received on 6LoWPAN
 * \param sourceport Source port
 * \param destport Destination port
 * \param payload Pointer to data in UDP packet
 * \param payloadlen Length of data in UDP packet
 * \param payloadmax The maximum size of the buffer which the user can write to
 * \param ipbuf Pointer to structure containing some info on the IP header
 * \return The size of data the user has written to the buffer
 *
 * The user can send a UDP message back to the source by putting the message in the
 * 'payload' buffer. The user must say how much data has been written to this buffer,
 * the stack will automatically switch dest/source addresses and UDP ports.
 *
*/
uint8_t sixlowpan_udp_usercall(uint16_t sourceport, uint16_t destport,
                            uint8_t * payload, uint8_t payloadlen,
                            uint8_t payloadmax, ipbuf_t * ipbuf,
                            ftData* rxFrame); // added to get access to originAddress

#endif /* __AVR_SIXLOWPAN_H__ */
/** @} */
/** @} */
