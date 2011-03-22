/* Copyright (c) 2008-2009 ATMEL Corporation
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
 * $Id: avr_sixlowpan.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 */
/**
 * @file
 *         AVR 6LoWPAN / IP Stack Implementation
 */
#include <stdio.h>
#include <string.h>
#include "../inc/avr_sixlowpan.h"
#include "../inc/avr_timer.h"
#include "../inc/deRFaddon/usb.h"
#include "../inc/mac.h"
#include "../inc/mac_data.h"
#include "../inc/mac_event.h"
#include "../inc/hal.h"
#include "../inc/hal_avr.h"
#include "../inc/system.h"
#include "../inc/sensn/app_interface.h"
#include "../inc/deRFaddon/uart.h"

//#include "sixlowpan_wake.h"

#if (IPV6LOWPAN == 1) || defined(DOXYGEN)

/**
 @addtogroup avr6lowpan
 @{

 This (optional) component of RUM adds several interesting features
 to the RUM stack.

 -# All wireless packets use a compressed version of IPV6 packets,
 called 6LoWPAN, as described by RFC4944.

 -# If the coordinator node is able to connect to the internet, each
 node on the wireless network is world addressable by a unique
 IPV6 address.  This means that sensor networks can span very
 large distances and form one virtual wireless network.

 To add this functionality to RUM, make sure the @ref IPV6LOWPAN flag is
 either left undefined during the compile, or it is defined as one.
 */

void macsixlowpanDataRequest(u16 addr, u8 len, u8 * data);

/**
 * Pointer to the MAC buffer
 */
uint8_t * mac_ptr;

/**
 * Used to keep track of the length of the 6LoWPAN header
 */
uint8_t mac_hdr_len;

/**
 * Number of contexts allowed in HC01
 */
#define NUM_PREFIXES 2

#define PRINTF(x)

#define FRAG_BUF               ((struct sixlowpan_frag_hdr *)mac_ptr)
#define HC1_BUF                ((struct sixlowpan_hc1_hdr *)(mac_ptr + mac_hdr_len))
#define HC1_HC_UDP_BUF         ((struct sixlowpan_hc1_hc_udp_hdr *)(mac_ptr + mac_hdr_len))
#define IP_BUF                 ((struct uip_ip_hdr *)(mac_ptr + mac_hdr_len))
#define IPHC_BUF               ((struct sixlowpan_iphc_hdr *)(mac_ptr + mac_hdr_len))

#define UIP_ND6_RS_BUF         ((struct uip_nd6_rs *)&mac_ptr[mac_hdr_len + UIP_ICMPH_LEN])
#define UIP_ND6_RA_BUF         ((struct uip_nd6_ra *)&mac_ptr[mac_hdr_len + UIP_ICMPH_LEN])

#define UIP_ND6_NS_BUF         ((struct uip_nd6_ns *)&mac_ptr[mac_hdr_len + UIP_ICMPH_LEN])
#define UIP_ND6_NA_BUF         ((struct uip_nd6_na *)&mac_ptr[mac_hdr_len + UIP_ICMPH_LEN])

#define UIP_ND6_OPT_LLAO_LEN   8

#define UIP_ND6_OPT_HDR_BUF    ((struct uip_nd6_opt_hdr *)&mac_ptr[mac_hdr_len + UIP_ICMPH_LEN + nd6_opt_offset])
#define UIP_ND6_OPT_PREFIX_BUF ((struct uip_nd6_opt_prefix_info *)&mac_ptr[mac_hdr_len + UIP_ICMPH_LEN + nd6_opt_offset])

#define UIP_ICMP_BUF           ((struct uip_icmp_hdr *)&mac_ptr[mac_hdr_len])

#define HTONS(n) (u16_t)((((u16_t) (n)) << 8) | (((u16_t) (n)) >> 8))

/** @brief MAC/IP Buffer used by AVR 6LoWPAN */
//uint8_t mac_buf[127];

ftData txBuf;
//ftData* rxBuf;

/** @brief Pointer to llao option in mac_buf */
static struct uip_nd6_opt_llao *nd6_opt_llao;

/** pointer to the byte where to write next inline field for HC01. */
static u8_t *hc01_ptr;

static uint16_t * hc01_chksum_ptr;

static uint16_t udpLocalAddr;

static u16_t chksum(u16_t sum, const u8_t *data, u16_t len);

uint8_t mac_len;

/** @brief Hold the context information used by HC01 */
sixlowpan_prefix_t context_prefix[NUM_PREFIXES];

extern void ledoff1(void);

#define NO_CONTEXT 0xff

uint8_t use_context = NO_CONTEXT;

/** @brief Initilize the 6LOWPAN layer
 *
 *  Performs any needed setup of variables / functions.
 *  Call before using any 6lowpan stuff.
 */
void sixlowpan_init(void) {
	//UART_PRINT("sixlowpan_init\r\n");
	context_prefix[0].checksum = 0xfe80;
	context_prefix[0].is_used = 1;

	//    sixlowpanSleep_init();
}

/** @brief Process an incoming 6lowpan frame that
 *  is compressed with HC01
 *
 *  Will call any output needed, such as answering a
 *  NS with a NA.
 *
 * This code makes a number of assumptions / requirements
 * about the 6lowpan format. These are the following:
 *
 *  - The IP address of this node will ALWAYS be totally
 *   compressed. That means the prefix is in the context
 *   list, the IID is based on the hardware address.
 *
 *  - The IP address which sends data to this node can
 *   be anything. It is never decompressed - the code just
 *   'turns it around' and sends back to that address.
 *
 *  - The code does not accept multicast packets, and will
 *   ignore them. Multicast is bad on a mesh network anyway.
 *
 *  - This code only GENERATES valid IPv6 checksums. It does
 *    nothing to verify that incoming packets have valid checksums.
 */

void sixlowpan_hc01_process(ftData* rxFrame, uint8_t payloadlen) {
	//UART_PRINT("sixlowpan_hc01_process\r\n");

	ipbuf_t ipbuf;

	mac_ptr = rxFrame->payload;
	mac_len = payloadlen;
	mac_hdr_len = 0;

	hc01_ptr = mac_ptr + 3;

	/* Dest address based on MAC address, Version, Flow, Traffic compressed */
	if ((IPHC_BUF->dispatch == SIXLOWPAN_DISPATCH_IPHC)
			&& ((IPHC_BUF->encoding[1] & 0x0C) == SIXLOWPAN_IPHC_DAM_0)) {
		//UART_PRINT("  -correct 6LoWPAN Message\r\n");

		/* Version & Flow Uncompressed */
		if ((IPHC_BUF->encoding[0] & 0x40) == 0) {
			if ((IPHC_BUF->encoding[0] & 0x80) == 0) {
				/* Traffic class is carried inline */
				hc01_ptr += 4;
			} else {
				/* Traffic class is compressed */
				hc01_ptr += 3;
			}

			/* Version & Flow Compressed */
		} else {
			if ((IPHC_BUF->encoding[0] & 0x80) == 0) {
				/* Traffic class is carried inline */
				hc01_ptr += 1;
			} else {
				/* Traffic class is compressed */
				;
			}
		}

		/* Next-header field */
		if (IPHC_BUF->encoding[0] & SIXLOWPAN_IPHC_NH_C) {
			ipbuf.proto = UIP_PROTO_UDP; /* Currently only UDP compressed like that */
		} else {
			ipbuf.proto = *hc01_ptr;
			hc01_ptr++;
		}

		/* Remember location of TTL */
		ipbuf.ttlptr = hc01_ptr;

		/* Check TTL */
		switch (IPHC_BUF->encoding[0] & 0x18) {
		case SIXLOWPAN_IPHC_TTL_1:
			ipbuf.ttl = 1;
			break;
		case SIXLOWPAN_IPHC_TTL_64:
			ipbuf.ttl = 64;
			break;
		case SIXLOWPAN_IPHC_TTL_255:
			ipbuf.ttl = 255;
			break;
		case SIXLOWPAN_IPHC_TTL_I:
			ipbuf.ttl = *hc01_ptr;
			hc01_ptr += 1;
			break;
		}

		/* Destination address is context-based */
		ipbuf.destcontext = (IPHC_BUF->encoding[1] & 0x03);

		/* Source address could be anything (off-link even) */
		ipbuf.srcmode = (IPHC_BUF->encoding[1] & 0xC0) >> 4;
		ipbuf.srccontext = (IPHC_BUF->encoding[1] & 0x30) >> 4;

		/* We will just use source address as destination when sending
		 * anything back out, hence we never store the address. It will
		 * automatically be at the proper place in the packet. */
		ipbuf.srcptr = hc01_ptr;

		switch (ipbuf.srcmode) {
		case SIXLOWPAN_IPHC_DAM_0: /* Note: we use 'DAM' constants instead of 'SAM'
		 * since address mode is shifted right 4 in this var */
			break;
		case SIXLOWPAN_IPHC_DAM_16:
			if ((*hc01_ptr & 0x80) == 0) {
				/* unicast address */
				hc01_ptr += 2;
			} else {
				PRINTF("sixlowpan: Don't deal with multicast\n");
				return;
			}
			break;
		case SIXLOWPAN_IPHC_DAM_64:
			hc01_ptr += 8;
			break;
		case SIXLOWPAN_IPHC_DAM_I:
			hc01_ptr += 16;
			break;
		}

		mac_hdr_len = hc01_ptr - rxFrame->payload;
		ipbuf.mac_hdr_len = mac_hdr_len;

		/********** ICMP Message **********/
		if (ipbuf.proto == UIP_PROTO_ICMP6) {
			//TODO delete later
			UART_PRINT("  -ICMP Message\r\n");
			if (UIP_ICMP_BUF->type == ICMP6_ECHO_REQUEST) {
				//TODO delete later
				//UART_PRINT("  -ICMP ECHO REQUEST Message\r\n");
				LED_ON(1);
				macSetAlarm(LED_DELAY, ledoff1);

				mac_hdr_len = 0;

				/* Check if TTL is carried inline, we reset it to 64 if so... */
				if ((IPHC_BUF->encoding[0] & 0x18) == SIXLOWPAN_IPHC_TTL_I) {
					*(mac_ptr + 4) = 64;
				}

				/* Send packet back to source */
				IPHC_BUF->encoding[1] = SIXLOWPAN_IPHC_SAM_0
						| (ipbuf.destcontext << 4);
				IPHC_BUF->encoding[1] |= ipbuf.srcmode | ipbuf.srccontext;

				mac_hdr_len = ipbuf.mac_hdr_len;

				/* Send echo reply */
				UIP_ICMP_BUF->type = ICMP6_ECHO_REPLY;

				/* We cheat the checksum since we just changed echo request to echo reply... */
				UIP_ICMP_BUF->icmpchksum = UIP_ICMP_BUF->icmpchksum
						+ ~(HTONS(0x0000));

				//                sixlowpanSleep_activity();
				macsixlowpanDataRequest(DEFAULT_COORD_ADDR, payloadlen,
						rxFrame->payload);
			}

			if (UIP_ICMP_BUF->type == ICMP6_ECHO_REPLY) {
				sixlowpan_ping_usercall(*(hc01_ptr + 7));
			}

			if (UIP_ICMP_BUF->type == ICMP6_RA) {
				sixlowpan_hc01_process_ra(&ipbuf);
			}

			if (UIP_ICMP_BUF->type == ICMP6_NS) {
				/* Destination is sender */
				ipbuf.destcontext = ipbuf.srccontext;
				ipbuf.destmode = ipbuf.srcmode;

				/* Source context will be same */
				sixlowpan_hc01_gen_na(&ipbuf, 1);
			}
		}

		/********** UDP Message **********/
		if (ipbuf.proto == UIP_PROTO_UDP) {
			/* Figure out port, while swapping src/dest */
			uint16_t srcport, destport;

			if (*hc01_ptr == SIXLOWPAN_NHC_UDP_C) {
				hc01_ptr++;

				srcport = (*hc01_ptr >> 4) + SIXLOWPAN_UDP_PORT_MIN;
				destport = (*hc01_ptr & 0x0f) + SIXLOWPAN_UDP_PORT_MIN;

				/* Swap nibbles */
				uint8_t temp;
				temp = *hc01_ptr >> 4;
				temp |= ((*hc01_ptr) & 0x0f) << 4;

				/* Store swapped dest/src */
				*hc01_ptr = temp;

				hc01_ptr++;

				mac_hdr_len += 2;

			} else if (*hc01_ptr == SIXLOWPAN_NHC_UDP_I) {
				srcport = HTONS(*((uint16_t *)(hc01_ptr + 1)));
				destport = HTONS(*((uint16_t *)(hc01_ptr + 3)));

				/* Store swapped dest/src ports */
				*((uint16_t *) (hc01_ptr + 1)) = HTONS(destport);
				*((uint16_t *) (hc01_ptr + 3)) = HTONS(srcport);

				hc01_ptr += 5;
				mac_hdr_len += 5;

			} else {
				return;
			}

			hc01_chksum_ptr = (uint16_t *) hc01_ptr;

			/* Add checksum of source and destination */
			*hc01_chksum_ptr = srcport + destport;

			/* Check for overflow... */
			if ((*hc01_chksum_ptr < srcport) || (*hc01_chksum_ptr < destport)) {
				(*hc01_chksum_ptr)++;
			}

			hc01_ptr += 2;
			mac_hdr_len += 2;

			/* Calculate maximum allowed payload */
			uint8_t maxPayload;
			maxPayload = 113 - mac_hdr_len; /* RUM allows 113 byte payloads */

			/* Call user program */
			uint8_t userdata;
			userdata = sixlowpan_udp_usercall(srcport, destport, hc01_ptr,
					mac_len - mac_hdr_len, maxPayload, &ipbuf, rxFrame); // added to get access to originAddr

			/* If user wants to send data out, do so */
			if (userdata) {
				//TODO delete later
				//UART_PRINT("  UDP packet to send\r\n");

				ipbuf.mac_hdr_len = mac_hdr_len;
				mac_hdr_len = 0;

				/* Reset TTL */
				if ((IPHC_BUF->encoding[0] & 0x18) == SIXLOWPAN_IPHC_TTL_I) {
					*(ipbuf.ttlptr) = 64;
				} else {
					IPHC_BUF->encoding[0] = (IPHC_BUF->encoding[0] & ~(0x18))
							| SIXLOWPAN_IPHC_TTL_64;
				}

				/* Send packet back to source */
				IPHC_BUF->encoding[1] = SIXLOWPAN_IPHC_SAM_0
						| (ipbuf.destcontext << 4); //New Source
				IPHC_BUF->encoding[1] |= ipbuf.srcmode | ipbuf.srccontext; //New Dest

				mac_hdr_len = ipbuf.mac_hdr_len;

				/* Figure out new UDP packet length */
				mac_len = userdata + mac_hdr_len;

				/* Send over radio */
				sixlowpan_hc01_udp_send();
			}
		}
	}

}

/** @brief Process a router advertisement
 *
 *  Read the prefix from the RA. If we  do not yet
 *  have 'context' for the 6lowpan network, which
 *  means we don't have a prefix, this routine
 *  will save it. Note the actual prefix is never
 *  stored, just the checksum of the prefix.
 */
void sixlowpan_hc01_process_ra(ipbuf_t * ipbuf) {
	UART_PRINT("sixlowpan_hc01_process_ra\r\n");


	struct uip_nd6_opt_prefix_info * nd6_opt_prefix_info;
	uint8_t nd6_opt_offset;

	/* Very basic sanity check */
	if (ipbuf->ttl != 255)
		return;

	/* Find prefix option */
	nd6_opt_offset = UIP_ND6_RA_LEN;

	while ((mac_hdr_len + 4 + nd6_opt_offset) < mac_len) {
		if (UIP_ND6_OPT_HDR_BUF->len == 0) {
			return;
		}

		/* Is this option prefix option? */
		if (UIP_ND6_OPT_HDR_BUF->type == UIP_ND6_OPT_PREFIX_INFO) {
			nd6_opt_prefix_info
					= (struct uip_nd6_opt_prefix_info *) UIP_ND6_OPT_HDR_BUF;

			uint8_t i = 1;

			uint16_t prefixchecksum;

			prefixchecksum = chksum(0, nd6_opt_prefix_info->prefix, 16);

			/* We only ever need to store the checksum of the prefix - no need
			 * to ever know. We need the checksum for adding in the ICMP checksum */
			if (!context_prefix[i].is_used) {
				context_prefix[i].checksum = prefixchecksum;
				context_prefix[i].is_used = 1;

				use_context = i;
			}

			/* This code can send unsolicited RA's - however it's of no real use in this network */
#if 0
			/* We have an IP address associated with this context - we should advertise it
			 * since we can avoid needing to store it this way.
			 */
			if (prefixchecksum == context_prefix[i].checksum)
			{
				ipbuf_t ipbuf;
				ipbuf.destcontext = 0;
				ipbuf.destmode = SIXLOWPAN_IPHC_DAM_16;
				ipbuf.srccontext = i;
				ipbuf.srcmode = SIXLOWPAN_IPHC_SAM_0;

				/* Destination is all-routers multicast address */
				/* 3 first bits = 101 */
				mac_ptr[4] = SIXLOWPAN_IPHC_MCAST_RANGE | (0x02) << 1;
				/* bits 3-6 = scope = bits 8-11 in 128 bits address */

				/*
				 * bits 7 - 15 = 9-bit group
				 * All-routers group
				 */
				mac_ptr[5] = 0x02;

				uint8_t * targetptr;

				ipbuf.mac_hdr_len = 6;

				/* 4 is length of HC01 header, 4 is length between ICMP and target */
				targetptr = &mac_ptr[UIP_ICMPH_LEN + ipbuf.mac_hdr_len + 4];

				uint8_t j;
				/* Copy prefix lower in memory. The RA will always be longer than the NS, so
				 * this will always work */
				for(j = 0; j < 8; j++) {
					*targetptr = nd6_opt_prefix_info->prefix[j];
					targetptr++;
				}

				/* Setup our address */
				*targetptr++ = MSB(macConfig.panId) | 0x02;
				*targetptr++ = LSB(macConfig.panId);
				*targetptr++ = 0x00;
				*targetptr++ = 0xff;
				*targetptr++ = 0xfe;
				*targetptr++ = 0x00;
				*targetptr++ = MSB(macConfig.shortAddress);
				*targetptr++ = LSB(macConfig.shortAddress);

				/* Setup rest of NA crapola */
				sixlowpan_hc01_gen_na(&ipbuf, 0);

			}
#endif

			/* Only check first prefix we find... */
			break;
		}

		nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
	}
}

/** @brief Generate a Neighbor Advertisement
 *  @param ipbuf Sets up source/dest and HC01 options
 *  @param solicited This value is copied to the 'solicited'
 *                   flag in the NA.
 *
 *  The source is always based on our MAC address, where the
 *  prefix is specified by the ipbuf->srccontext option.
 *
 *  The destination should be set up by ipbuf->destcontext and
 *  ipbuf->destmode - any needed setup of the destination IP
 *  address should already be setup in the buffer.
 */
void sixlowpan_hc01_gen_na(ipbuf_t * ipbuf, uint8_t solicited) {
	//UART_PRINT("sixlowpan_hc01_gen_na\r\n");
	mac_hdr_len = 0;
	mac_ptr = txBuf.payload;

	hc01_ptr = mac_ptr + 3;

	IPHC_BUF->dispatch = SIXLOWPAN_DISPATCH_IPHC;
	IPHC_BUF->encoding[0] = 0;
	IPHC_BUF->encoding[1] = 0;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_VF_C;
	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TC_C;

	*hc01_ptr = UIP_PROTO_ICMP6;
	hc01_ptr += 1;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TTL_255;

	/* Source IP address */
	IPHC_BUF->encoding[1] |= (ipbuf->srccontext) << 4;
	/* elide the IID */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_SAM_0;

	/* Destination is sender */
	IPHC_BUF->encoding[1] |= ipbuf->destcontext;
	IPHC_BUF->encoding[1] |= ipbuf->destmode;

	mac_hdr_len = ipbuf->mac_hdr_len;

	/* Neighbor Advertisement */
	UIP_ICMP_BUF->type = ICMP6_NA;
	UIP_ICMP_BUF->icode = 0;

	UIP_ND6_NA_BUF->reserved[0] = 0;
	UIP_ND6_NA_BUF->reserved[1] = 0;
	UIP_ND6_NA_BUF->reserved[2] = 0;

	/* This code sends solicited NA */
	if (solicited) {
		UIP_ND6_NA_BUF->flagsreserved = UIP_ND6_NA_FLAG_SOLICITED;
	} else {
		UIP_ND6_NA_BUF->flagsreserved = 0;
	}

	/* Need to set the 'UIP_ND6_NA_BUF->tgtipaddr' - BUT since we are sending,
	 * solicitied responses, this is the same thing as in the NA. This lies at
	 * the same location in the NA too - we just don't do anything, and magically
	 * it is set up correctly */

	/* Target Link-Layer Address */
	mac_len = UIP_ICMPH_LEN + UIP_ND6_NA_LEN + UIP_ND6_OPT_LLAO_LEN
			+ mac_hdr_len;
	nd6_opt_llao = (struct uip_nd6_opt_llao *) &mac_ptr[UIP_ICMPH_LEN
			+ UIP_ND6_NA_LEN + mac_hdr_len];
	nd6_opt_llao->type = UIP_ND6_OPT_TLLAO; /* type of the option */
	nd6_opt_llao->len = 1; /* 8-bytes */
	nd6_opt_llao->addr[0] = MSB(macConfig.shortAddress);
	nd6_opt_llao->addr[1] = LSB(macConfig.shortAddress);

	/* Pad with zeros */
	uint8_t i;
	for (i = 2; i < 6; i++) {
		nd6_opt_llao->addr[i] = 0x00;
	}

	/* ICMP Checksum */
	/*
	 * The checksum is the 16-bit one's complement of the one's complement
	 * sum of the entire ICMPv6 message starting with the ICMPv6 message
	 * type field, prepended with a "pseudo-header" of IPv6 header fields,
	 * as specified in [IPv6, section 8.1].
	 *
	 */
	UIP_ICMP_BUF->icmpchksum = 0;
	uint16_t checksum16 = sixlowpan_hc01_pseudochksum();

	/* Sum all the ICMP stuff */
	checksum16 = chksum(checksum16, mac_ptr + mac_hdr_len, mac_len
			- mac_hdr_len);
	UIP_ICMP_BUF->icmpchksum = ~HTONS(checksum16);

	//    sixlowpanSleep_activity();
	macsixlowpanDataRequest(DEFAULT_COORD_ADDR, mac_len, mac_ptr);
}
/**
 *  @brief Sends a router solicitation
 *
 *  The code will generate a multicast packet
 *  directed to the ff02::2 address (all-routers) for the
 *  RS.
 *
 *  The link-layer address will be the coordinator, which
 *  should respond with a router advertisement.
 */
void sixlowpan_hc01_gen_rs(void) {

	//UART_PRINT("sixlowpan_hc01_gen_rs\r\n");

	mac_hdr_len = 0;
	mac_ptr = txBuf.payload;

	hc01_ptr = mac_ptr + 3;

	IPHC_BUF->dispatch = SIXLOWPAN_DISPATCH_IPHC;
	IPHC_BUF->encoding[0] = 0;
	IPHC_BUF->encoding[1] = 0;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_VF_C;
	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TC_C;

	*hc01_ptr = UIP_PROTO_ICMP6;
	hc01_ptr += 1;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TTL_255;

	/* Source IP address: Link-local prefix */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_ADDR_CONTEXT_LL << 4;
	/* elide the IID */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_SAM_0;

	/* Destination is all-routers multicast address */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_DAM_16;
	/* 3 first bits = 101 */
	*hc01_ptr = SIXLOWPAN_IPHC_MCAST_RANGE;
	/* bits 3-6 = scope = bits 8-11 in 128 bits address */
	*hc01_ptr |= (0x02) << 1;
	/*
	 * bits 7 - 15 = 9-bit group
	 * All-routers group
	 */
	*(hc01_ptr + 1) = 0x02;
	hc01_ptr += 2;

	mac_hdr_len = 6;

	/* Router solicitation */
	UIP_ICMP_BUF->type = ICMP6_RS;
	UIP_ICMP_BUF->icode = 0;
	UIP_ND6_RS_BUF->reserved = 0;

	/* Source Link-Layer Address */
	mac_len = UIP_ICMPH_LEN + UIP_ND6_RS_LEN + UIP_ND6_OPT_LLAO_LEN
			+ mac_hdr_len;
	nd6_opt_llao = (struct uip_nd6_opt_llao *) &mac_ptr[UIP_ICMPH_LEN
			+ UIP_ND6_RS_LEN + mac_hdr_len];
	nd6_opt_llao->type = UIP_ND6_OPT_SLLAO; /* type of the option */
	nd6_opt_llao->len = 1; /* 8-bytes */
	nd6_opt_llao->addr[0] = MSB(macConfig.shortAddress);
	nd6_opt_llao->addr[1] = LSB(macConfig.shortAddress);

	/* Pad with zeros */
	uint8_t i;
	for (i = 2; i < 6; i++) {
		nd6_opt_llao->addr[i] = 0x00;
	}

	/* ICMP Checksum */
	/*
	 * The checksum is the 16-bit one's complement of the one's complement
	 * sum of the entire ICMPv6 message starting with the ICMPv6 message
	 * type field, prepended with a "pseudo-header" of IPv6 header fields,
	 * as specified in [IPv6, section 8.1].
	 *
	 */

	UIP_ICMP_BUF->icmpchksum = 0;
	uint16_t checksum16 = sixlowpan_hc01_pseudochksum();

	/* Sum all the ICMP stuff */
	checksum16 = chksum(checksum16, mac_ptr + mac_hdr_len, mac_len
			- mac_hdr_len);
	UIP_ICMP_BUF->icmpchksum = ~HTONS(checksum16);

	//    sixlowpanSleep_activity();
	macsixlowpanDataRequest(DEFAULT_COORD_ADDR, mac_len, mac_ptr);

}

/** @brief Setup an ICMPv6 Echo Request packet for off-link
 *          IP addresses
 *  @param sequence The sequence number in the echo request to send
 *  @return A pointer to where you must load the 16-byte
 *          IP address
 *
 *  The source is always based on our MAC address, where the
 *  destination must be off-link and loaded by the user.
 *
 *  As an example:
 *
 *  @code
 *      //Load 2001:0DB8::1 as ping destination, with seq = 26
 *      uint8_t * ipaddr = sixlowpan_hc01_ping_setup_ipglobal(26);
 *
 *      *(ipaddr + 0) = 0x20;
 *      *(ipaddr + 1) = 0x01;
 *      *(ipaddr + 2) = 0x0D;
 *      *(ipaddr + 3) = 0x0B;
 *      *(ipaddr + 4) = 0x08;
 *      ...
 *      *(ipaddr + 15) = 0x01;
 *
 *      sixlowpan_hc01_ping_send();
 *  @endcode
 *
 *
 */
uint8_t * sixlowpan_hc01_ping_setup_ipglobal(uint8_t sequence) {
	//UART_PRINT("sixlowpan_hc01_ping_setup_ipglobal\r\n");
	mac_hdr_len = 0;
	mac_ptr = txBuf.payload;

	uint8_t * ipptr;

	hc01_ptr = mac_ptr + 3;

	IPHC_BUF->dispatch = SIXLOWPAN_DISPATCH_IPHC;
	IPHC_BUF->encoding[0] = 0;
	IPHC_BUF->encoding[1] = 0;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_VF_C;
	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TC_C;

	*hc01_ptr = UIP_PROTO_ICMP6;
	hc01_ptr += 1;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TTL_64;

	/* Source IP address: Global prefix */
	IPHC_BUF->encoding[1] |= 1 << 4;
	/* elide the IID */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_SAM_0;

	/* Destination is fancy-boy address */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_DAM_I;

	ipptr = hc01_ptr;

	hc01_ptr += 16;
	mac_hdr_len = 20;

	/* Send echo reply */
	UIP_ICMP_BUF->type = ICMP6_ECHO_REQUEST;
	UIP_ICMP_BUF->icmpchksum = 0;
	UIP_ICMP_BUF->icode = 0;

	/* ICMP Header */
	hc01_ptr += 4;

	*hc01_ptr++ = 0; /* Identifier - not used */
	*hc01_ptr++ = 0;

	*hc01_ptr++ = 0; /* Sequence number */
	*hc01_ptr++ = sequence;

	/* Just send echo request, no data with it */
	mac_len = mac_hdr_len + 8;

	return ipptr;
}

/** @brief Send a Ping Packet
 *
 *  The UDP packet must already have been set up with
 *  a call to sixlowpan_hc01_ping_setup_ipxxx() and
 *  This routine adds the checksum and sends out the MAC
 *  layer.
 */
void sixlowpan_hc01_ping_send(void) {
	//UART_PRINT("sixlowpan_hc01_ping_send\r\n");
	/* Check we have context before doing anything... */
	if (use_context == NO_CONTEXT) {
		//TODO remove comment
		//sixlowpan_hc01_gen_rs();
		//return;
	}

	/* Pseudo-header Checksum */
	uint16_t checksum16 = sixlowpan_hc01_pseudochksum();

	/* Sum all the payload */
	checksum16 = chksum(checksum16, mac_ptr + mac_hdr_len, mac_len
			- mac_hdr_len);

	UIP_ICMP_BUF->icmpchksum = ~HTONS(checksum16);

	/* If result is zero, set to 0xffff */
	if (UIP_ICMP_BUF->icmpchksum == 0x0000) {
		UIP_ICMP_BUF->icmpchksum = 0xFFFF;
	}

	//    sixlowpanSleep_activity();
	macsixlowpanDataRequest(DEFAULT_COORD_ADDR, mac_len, mac_ptr);
}

/** @brief Setup a UDP packet for off-link IP addresses
 *  @return A pointer to where you must load the 16-byte
 *          IP address
 *
 *  The source is always based on our MAC address, where the
 *  destination must be off-link and loaded by the user.
 *
 *  As a full example:
 *
 *  @code
 *      //Load 2001:0DB8::1 as UDP destination
 *      uint8_t * ipaddr = sixlowpan_hc01_udp_setup_ipglobal();
 *
 *      *(ipaddr + 0) = 0x20;
 *      *(ipaddr + 1) = 0x01;
 *      *(ipaddr + 2) = 0x0D;
 *      *(ipaddr + 3) = 0x0B;
 *      *(ipaddr + 4) = 0x08;
 *      ...
 *      *(ipaddr + 15) = 0x01;
 *
 *      sixlowpan_hc01_udp_setup_ports(UDP_APP_PORT1, UDP_APP_PORT2);
 *      memcpy( sixlowpan_hc01_udp_get_payloadptr(), data, len);
 *      sixlowpan_hc01_udp_set_payloadsize(len);
 *      sixlowpan_hc01_udp_send();
 *  @endcode
 */
uint8_t * sixlowpan_hc01_udp_setup_ipglobal(void) {
	//UART_PRINT("sixlowpan_hc01_udp_setup_ipglobal\r\n");
	mac_hdr_len = 0;
	mac_ptr = txBuf.payload;

	uint8_t * ipptr;

	udpLocalAddr = DEFAULT_COORD_ADDR;

	hc01_ptr = mac_ptr + 3;

	IPHC_BUF->dispatch = SIXLOWPAN_DISPATCH_IPHC;
	IPHC_BUF->encoding[0] = 0;
	IPHC_BUF->encoding[1] = 0;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_VF_C;
	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TC_C;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_NH_C;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TTL_64;

	/* Source IP address: Global prefix */
	IPHC_BUF->encoding[1] |= 1 << 4;
	/* elide the IID */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_SAM_0;

	/* Destination is fancy-boy address */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_DAM_I;

	ipptr = hc01_ptr;

	hc01_ptr += 16;
	mac_hdr_len = 19;
	mac_len = mac_hdr_len;

	return ipptr;
}

/** @brief Setup a UDP packet for on-link IP addresses
 *  @param addr Address of the node to send to
 *
 *  The source is always based on our MAC address, where the
 *  destination must be on-link. As an example:
 *
 *  @code
 *      //Load 2001:0DB8::4 as UDP destination (assuming that is our network)
 *      uint8_t * ipaddr = sixlowpan_hc01_udp_setup_local(0x0004);
 *      sixlowpan_hc01_udp_setup_ports(UDP_APP_PORT1, UDP_APP_PORT2);
 *      memcpy( sixlowpan_hc01_udp_get_payloadptr(), data, len);
 *      sixlowpan_hc01_udp_set_payloadsize(len);
 *      sixlowpan_hc01_udp_send();
 *  @endcode
 *
 */
void sixlowpan_hc01_udp_setup_iplocal(uint16_t addr) {
	//UART_PRINT("sixlowpan_hc01_udp_setup_iplocal\r\n");
	mac_hdr_len = 0;
	mac_ptr = txBuf.payload;

	udpLocalAddr = addr;

	hc01_ptr = mac_ptr + 3;

	IPHC_BUF->dispatch = SIXLOWPAN_DISPATCH_IPHC;
	IPHC_BUF->encoding[0] = 0;
	IPHC_BUF->encoding[1] = 0;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_VF_C;
	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TC_C;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_NH_C;

	IPHC_BUF->encoding[0] |= SIXLOWPAN_IPHC_TTL_64;

	/* Source IP address: Global prefix */
	IPHC_BUF->encoding[1] |= 1 << 4;
	/* elide the IID */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_SAM_0;

	/* Source IP address: Global prefix */
	IPHC_BUF->encoding[1] |= 1;
	/* elide the IID */
	IPHC_BUF->encoding[1] |= SIXLOWPAN_IPHC_DAM_0;

	mac_hdr_len = 3;
	mac_len = mac_hdr_len;

	return;
}

/** @brief Setup a UDP packet's port information
 *  @param srcport Source port number
 *  @param destport Destination port number
 *
 *  The UDP packet must already have been set up with
 *  a call to sixlowpan_hc01_setup_ipxxx(), or the hc01_ptr
 *  must be manually set to point to the proper point in the
 *  6lowpan buffer.
 */
void sixlowpan_hc01_udp_setup_ports(uint16_t srcport, uint16_t destport) {
	//UART_PRINT("sixlowpan_hc01_udp_setup_ports\r\n");
	/* UDP Port Calculations */
	if (srcport >= SIXLOWPAN_UDP_PORT_MIN && srcport < SIXLOWPAN_UDP_PORT_MAX
			&& destport >= SIXLOWPAN_UDP_PORT_MIN && destport
			< SIXLOWPAN_UDP_PORT_MAX) {
		/* we can compress. Copy compressed ports, full chcksum */
		*hc01_ptr = SIXLOWPAN_NHC_UDP_C;
		*(hc01_ptr + 1) = (u8_t) ((srcport - SIXLOWPAN_UDP_PORT_MIN) << 4)
				+ (u8_t) ((destport - SIXLOWPAN_UDP_PORT_MIN));

		hc01_ptr += 2;
		mac_hdr_len += 2;
	} else {
		/* we cannot compress. Copy uncompressed ports, full chcksum */
		*hc01_ptr = SIXLOWPAN_NHC_UDP_I;
		*((uint16_t *) (hc01_ptr + 1)) = HTONS(srcport);
		*((uint16_t *) (hc01_ptr + 3)) = HTONS(destport);

		hc01_ptr += 5;
		mac_hdr_len += 5;
	}

	hc01_chksum_ptr = (uint16_t *) hc01_ptr;

	/* Add checksum of source and destination */
	*hc01_chksum_ptr = srcport + destport;

	/* Check for overflow... */
	if ((*hc01_chksum_ptr < srcport) || (*hc01_chksum_ptr < destport)) {
		(*hc01_chksum_ptr)++;
	}

	hc01_ptr += 2;
	mac_hdr_len += 2;

	mac_len = mac_hdr_len;
}

/** @brief Send a UDP Packet
 *
 *  The UDP packet must already have been set up with
 *  a call to sixlowpan_hc01_setup_ipxxx() and
 *  sixlowpan_hc01_setup_ports(). This routine adds
 *  the checksum and sends out the MAC layer.
 */
void sixlowpan_hc01_udp_send(void) {
	//UART_PRINT("sixlowpan_hc01_udp_send\r\n");
	/* Check we have context before doing anything... */
	if (use_context == NO_CONTEXT) {
		//TODO delete comment
		//sixlowpan_hc01_gen_rs();
		//return;
	}

	/* Pseudo-header Checksum */
	uint16_t checksum16 = sixlowpan_hc01_pseudochksum();

	/* UDP Header */
	uint32_t checksum = *hc01_chksum_ptr; /* Checksum of ports */
	checksum += checksum16; /* IPv6 Pseudo Checksum */
	checksum += mac_len - mac_hdr_len + 8; /* UDP field: payload length including UDP header */

	/* Add carries to 16-bit checksum */
	while (checksum & 0xFFFF0000) {
		checksum = (uint16_t) (checksum) + (uint16_t) (checksum >> 16);
	}

	checksum16 = checksum;

	/* Sum all the UDP payload */
	checksum16 = chksum(checksum16, mac_ptr + mac_hdr_len, mac_len
			- mac_hdr_len);

	*hc01_chksum_ptr = ~HTONS(checksum16);

	/* If result is zero, set to 0xffff */
	if (*hc01_chksum_ptr == 0x0000) {
		*hc01_chksum_ptr = 0xFFFF;
	}

	//    sixlowpanSleep_activity();
	macsixlowpanDataRequest(udpLocalAddr, mac_len, mac_ptr);
}

/** @brief Get a pointer to the payload section of the UDP packet
 *
 *  The use can use this to load a payload into the sixlowpan
 *  buffer directly.
 */
uint8_t * sixlowpan_hc01_udp_get_payloadptr(void) {
	return hc01_ptr;
}

/** @brief Set amount of data user has loaded into payload section
 *
 */
void sixlowpan_hc01_udp_set_payloadsize(uint8_t size) {
	hc01_ptr += size;
	mac_len += size;
}

/**
 * @brief Calculates the pseduo-header checksum
 * @returns checksum
 *
 * IPv6 does not have a checksum on it's header.
 * Hence higher-level layers include a 'pseudo-header'
 * that they calculate the checksum for. This
 * includes the source IP addr, destination IP addr,
 * next-header field, and packet length. This
 * routine calculates what the checksum should be for
 * a HC01 packet, but without actually decompressing
 * the entire packet (which would waste space).
 */
uint16_t sixlowpan_hc01_pseudochksum(void) {
	//UART_PRINT("sixlowpan_hc01_pseudochksum\r\n");
	uint32_t checksum = 0;

	uint8_t i;

	uint8_t * hc01_ptr_temp;
	uint8_t mac_hdr_len_temp;

	/* This will be restored at the end */
	mac_hdr_len_temp = mac_hdr_len;
	hc01_ptr_temp = hc01_ptr;

	hc01_ptr = mac_ptr + 3;
	mac_hdr_len = 0;

	/* These don't affect checksum, just location */
	/* Version & Flow Uncompressed */
	if ((IPHC_BUF->encoding[0] & 0x40) == 0) {
		if ((IPHC_BUF->encoding[0] & 0x80) == 0) {
			/* Traffic class is carried inline */
			hc01_ptr += 4;
		} else {
			/* Traffic class is compressed */
			hc01_ptr += 3;
		}
	} else
	/* Version & Flow Compressed */
	{
		if ((IPHC_BUF->encoding[0] & 0x80) == 0) {
			/* Traffic class is carried inline */
			hc01_ptr += 1;
		} else {
			/* Traffic class is compressed */
			;
		}
	}

	/* Next-header field */
	if (IPHC_BUF->encoding[0] & SIXLOWPAN_IPHC_NH_C) {
		checksum += UIP_PROTO_UDP; /* Currently only UDP compressed like that */
		checksum += 8; /* UDP has 8 bytes in UDP header, this will not be calculated
		 properly later when 'upper layer length' is found, so we
		 add them here */
	} else {
		checksum += *hc01_ptr;
		hc01_ptr++;
	}

	/* Source address IID is ALWAYS based on MAC address */
	checksum += context_prefix[(IPHC_BUF->encoding[1] & 0x30) >> 4].checksum;
	checksum += (macConfig.panId | (uint16_t) 0x0200);
	checksum += 0xfe00;
	checksum += 0x00ff;
	checksum += macConfig.shortAddress;

	/* Check if TTL is uncompressed... */
	if ((IPHC_BUF->encoding[0] & 0x18) == SIXLOWPAN_IPHC_TTL_I) {
		hc01_ptr++;
	}

	/* Dest address could be anything... more complicated! */
	switch (IPHC_BUF->encoding[1] & 0x0C) {
	case SIXLOWPAN_IPHC_DAM_0:

		/* IID Based on MAC Address */
		checksum += context_prefix[(IPHC_BUF->encoding[1] & 0x03)].checksum;
		checksum += (macConfig.panId | (uint16_t) 0x0200);
		checksum += 0xfe00;
		checksum += 0x00ff;
		checksum += 0x0000; /* Router is always address 0 we assume!! */
		break;

	case SIXLOWPAN_IPHC_DAM_16:
		if ((*hc01_ptr & 0x80) == 0) {
			/* unicast address */
			checksum += context_prefix[(IPHC_BUF->encoding[1] & 0x03)].checksum;
			checksum += (*hc01_ptr << 8) | (*(hc01_ptr + 1));
			hc01_ptr += 2;
		} else {
			/* Multicast address - we just assume we know the multicast group */
			checksum += 0xFF00;
			checksum += ((*hc01_ptr >> 1) & 0x0F);
			checksum += *(hc01_ptr + 1);
			hc01_ptr += 2;
		}
		break;
	case SIXLOWPAN_IPHC_DAM_64:
		/* Add lower 64-bits */
		checksum += context_prefix[(IPHC_BUF->encoding[1] & 0x03)].checksum;
		for (i = 0; i < 8; i += 2) {
			checksum += (uint16_t) (*(hc01_ptr + i) << 8)
					+ (uint16_t) (*(hc01_ptr + i + 1));
		}

		hc01_ptr += 8;
		break;
	case SIXLOWPAN_IPHC_DAM_I:
		for (i = 0; i < 16; i += 2) {
			checksum += (uint16_t) (*(hc01_ptr + i) << 8)
					+ (uint16_t) (*(hc01_ptr + i + 1));
		}
		hc01_ptr += 16;
		break;
	}

	/* Restore pointer */
	hc01_ptr = hc01_ptr_temp;
	mac_hdr_len = mac_hdr_len_temp;

	/* Upper length packet length */
	checksum += mac_len - mac_hdr_len;

	/* Add carries to 16-bit checksum */
	while (checksum & 0xFFFF0000) {
		checksum = (uint16_t) (checksum) + (uint16_t) (checksum >> 16);
	}

	return (uint16_t) checksum;

}

/**
 * @brief Calculates an IP checksum over a range of data
 * @param sum Starting sum
 * @param data Pointer to data to checksum
 * @param len Length of data to checksum
 * @returns checksum
 *
 * Calculates 16-bit checksum for use in ICMPv6 etc.
 */
static u16_t chksum(u16_t sum, const u8_t *data, u16_t len) {
	u16_t t;
	const u8_t *dataptr;
	const u8_t *last_byte;

	dataptr = data;
	last_byte = data + len - 1;

	while (dataptr < last_byte) { /* At least two more bytes */
		t = (dataptr[0] << 8) + dataptr[1];
		sum += t;

		if (sum < t) {
			sum++; /* carry */
		}

		dataptr += 2;
	}

	if (dataptr == last_byte) {
		t = (dataptr[0] << 8) + 0;
		sum += t;

		if (sum < t) {
			sum++; /* carry */
		}
	}

	/* Return sum in host byte order. */
	return sum;
}

/**
 * @brief This routine is called from the MAC layer
 */
void sixlowpan_DataIndication(ftData *frame, uint8_t payloadlen) {
	//    sixlowpanSleep_activity();
	//UART_PRINT("sixlowpan_DataIndication\r\n");

	//memcpy(mac_buf, frame->payload, payloadlen);

	//mac_len = payloadlen;
	sixlowpan_hc01_process(frame, payloadlen);

	return;
}

#else
void sixlowpan_DataIndication(ftData *frame, uint8_t payloadlen)
{
}

void sixlowpan_init(void)
{
}

#endif  // #if (IPV6LOWPAN == 1) || defined(DOXYGEN)
/** @} */

