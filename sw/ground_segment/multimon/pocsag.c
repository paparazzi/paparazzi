/*
 *      pocsag.c -- POCSAG protocol decoder
 *
 *      Copyright (C) 1996  
 *          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
 *
 *      POCSAG (Post Office Code Standard Advisory Group)
 *      Radio Paging Decoder
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* ---------------------------------------------------------------------- */

#include "multimon.h"
#include <string.h>

/* ---------------------------------------------------------------------- */

#define CHARSET_LATIN1

/* ---------------------------------------------------------------------- */

/*
 * the code used by POCSAG is a (n=31,k=21) BCH Code with dmin=5,
 * thus it could correct two bit errors in a 31-Bit codeword.
 * It is a systematic code.
 * The generator polynomial is: 
 *   g(x) = x^10+x^9+x^8+x^6+x^5+x^3+1
 * The parity check polynomial is: 
 *   h(x) = x^21+x^20+x^18+x^16+x^14+x^13+x^12+x^11+x^8+x^5+x^3+1
 * g(x) * h(x) = x^n+1
 */
#define BCH_POLY 03551 /* octal */
#define BCH_N    31
#define BCH_K    21

/*
 * some codewords with special POCSAG meaning
 */
#define POCSAG_SYNC     0x7cd215d8
#define POCSAG_SYNCINFO 0x7cf21436
#define POCSAG_IDLE     0x7a89c197

#define POCSAG_SYNC_WORDS ((2000000 >> 3) << 13)

/* ---------------------------------------------------------------------- */

static unsigned char service_mask = 0x87;

/* ---------------------------------------------------------------------- */

static inline unsigned char even_parity(unsigned long data) 
{
	unsigned int temp = data ^ (data >> 16);
	
	temp = temp ^ (temp >> 8);
	temp = temp ^ (temp >> 4);
	temp = temp ^ (temp >> 2);
	temp = temp ^ (temp >> 1);
	return temp & 1;
}

/* ---------------------------------------------------------------------- */

static unsigned long pocsag_code(unsigned long data) 
{
	unsigned long ret = data << (BCH_N-BCH_K), shreg = ret;
	unsigned long mask = 1L << (BCH_N-1), coeff = BCH_POLY << (BCH_K-1);
	int n = BCH_K;
	
	for(; n > 0; mask >>= 1, coeff >>= 1, n--)
		if (shreg & mask)
			shreg ^= coeff;
	ret ^= shreg;
	ret = (ret << 1) | even_parity(ret);
	verbprintf(9, "BCH coder: data: %08lx shreg: %08lx ret: %08lx\n", 
		   data, shreg, ret);
	return ret;
}

/* ---------------------------------------------------------------------- */

static unsigned int pocsag_syndrome(unsigned long data) 
{
	unsigned long shreg = data >> 1; /* throw away parity bit */
	unsigned long mask = 1L << (BCH_N-1), coeff = BCH_POLY << (BCH_K-1);
	int n = BCH_K;
	
	for(; n > 0; mask >>= 1, coeff >>= 1, n--)
		if (shreg & mask)
			shreg ^= coeff;
	if (even_parity(data))
		shreg |= (1 << (BCH_N - BCH_K));
	verbprintf(9, "BCH syndrome: data: %08lx syn: %08lx\n", data, shreg);
	return shreg;
}

/* ---------------------------------------------------------------------- */

static void print_msg_numeric(struct l2_pocsag_rx *rx) 
{
	static const char *conv_table = "084 2.6]195-3U7[";
	unsigned char *bp = rx->buffer;
	int len = rx->numnibbles;
	char buf[256], *cp = buf;
       
	if (len >= sizeof(buf)) 
		len = sizeof(buf)-1;
	for (; len > 0; bp++, len -= 2) {
		*cp++ = conv_table[(*bp >> 4) & 0xf];
		if (len > 1)
			*cp++ = conv_table[*bp & 0xf];
	}
	*cp = '\0';
	verbprintf(0, "%s\n", buf);
}

/* ---------------------------------------------------------------------- */

static char *translate_alpha(unsigned char chr) 
{
	static const struct trtab {
		unsigned char code;
		char *str;
	} trtab[] = {{  0, "<NUL>" },
		     {  1, "<SOH>" }, 
		     {  2, "<STX>" }, 
		     {  3, "<ETX>" }, 
		     {  4, "<EOT>" }, 
		     {  5, "<ENQ>" }, 
		     {  6, "<ACK>" }, 
		     {  7, "<BEL>" }, 
		     {  8, "<BS>" }, 
		     {  9, "<HT>" }, 
		     { 10, "<LF>" }, 
		     { 11, "<VT>" }, 
		     { 12, "<FF>" }, 
		     { 13, "<CR>" }, 
		     { 14, "<SO>" }, 
		     { 15, "<SI>" }, 
		     { 16, "<DLE>" }, 
		     { 17, "<DC1>" }, 
		     { 18, "<DC2>" }, 
		     { 19, "<DC3>" }, 
		     { 20, "<DC4>" }, 
		     { 21, "<NAK>" }, 
		     { 22, "<SYN>" }, 
		     { 23, "<ETB>" }, 
		     { 24, "<CAN>" }, 
		     { 25, "<EM>" }, 
		     { 26, "<SUB>" }, 
		     { 27, "<ESC>" }, 
		     { 28, "<FS>" }, 
		     { 29, "<GS>" }, 
		     { 30, "<RS>" }, 
		     { 31, "<US>" }, 
#ifdef CHARSET_LATIN1
		     { 0x5b, "\304" }, /* upper case A dieresis */
		     { 0x5c, "\326" }, /* upper case O dieresis */
		     { 0x5d, "\334" }, /* upper case U dieresis */ 
		     { 0x7b, "\344" }, /* lower case a dieresis */
		     { 0x7c, "\366" }, /* lower case o dieresis */ 
		     { 0x7d, "\374" }, /* lower case u dieresis */ 
		     { 0x7e, "\337" }}; /* sharp s */
#else /* CHARSET_LATIN1 */
		     { 0x5b, "AE" }, /* upper case A dieresis */
		     { 0x5c, "OE" }, /* upper case O dieresis */
		     { 0x5d, "UE" }, /* upper case U dieresis */ 
		     { 0x7b, "ae" }, /* lower case a dieresis */
		     { 0x7c, "oe" }, /* lower case o dieresis */ 
		     { 0x7d, "ue" }, /* lower case u dieresis */ 
		     { 0x7e, "ss" }}; /* sharp s */
#endif /* CHARSET_LATIN1 */
	int min = 0, max = (sizeof(trtab) / sizeof(trtab[0])) - 1;
	
	/*
	 * binary search, list must be ordered!
	 */
	for (;;) {
		int mid = (min+max) >> 1;
		const struct trtab *tb = trtab + mid;
		int cmp = ((int) tb->code) - ((int) chr);

		if (!cmp)
			return tb->str;
		if (cmp < 0) {
			min = mid+1;
			if (min > max)
				return NULL;
		}
		if (cmp > 0) {
			max = mid-1;
			if (max < min)
				return NULL;
		}
	}
}				

/* ---------------------------------------------------------------------- */

static void print_msg_alpha(struct l2_pocsag_rx *rx) 
{
	unsigned long data = 0;
	int datalen = 0;
	unsigned char *bp = rx->buffer;
	int len = rx->numnibbles;
	char buf[256], *cp = buf;
	int buffree = sizeof(buf)-1;
	unsigned char curchr;
	char *tstr;
	
	while (len > 0) {
		while (datalen < 7 && len > 0) {
			if (len == 1) {
				data = (data << 4) | ((*bp >> 4) & 0xf);
				datalen += 4;
				len = 0;
			} else {
				data = (data << 8) | *bp++;
				datalen += 8;
				len -= 2;
			}
		}
		if (datalen < 7)
			continue;
		datalen -= 7;
		curchr = ((data >> datalen) & 0x7f) << 1;
		curchr = ((curchr & 0xf0) >> 4) | ((curchr & 0x0f) << 4);
		curchr = ((curchr & 0xcc) >> 2) | ((curchr & 0x33) << 2);
		curchr = ((curchr & 0xaa) >> 1) | ((curchr & 0x55) << 1);
		tstr = translate_alpha(curchr);
		if (tstr) {
			int tlen = strlen(tstr);
			if (buffree >= tlen) {
				memcpy(cp, tstr, tlen);
				cp += tlen;
				buffree -= tlen;
			}
		} else if (buffree > 0) {
			*cp++ = curchr;
			buffree--;
		}
	}
	*cp = '\0';
	verbprintf(0, "%s\n", buf);
}

/* ---------------------------------------------------------------------- */

static void print_msg_skyper(struct l2_pocsag_rx *rx) 
{
	unsigned long data = 0;
	int datalen = 0;
	unsigned char *bp = rx->buffer;
	int len = rx->numnibbles;
	char buf[256], *cp = buf;
	int buffree = sizeof(buf)-1;
	unsigned char curchr;
	char *tstr;
	
	while (len > 0) {
		while (datalen < 7 && len > 0) {
			if (len == 1) {
				data = (data << 4) | ((*bp >> 4) & 0xf);
				datalen += 4;
				len = 0;
			} else {
				data = (data << 8) | *bp++;
				datalen += 8;
				len -= 2;
			}
		}
		if (datalen < 7)
			continue;
		datalen -= 7;
		curchr = ((data >> datalen) & 0x7f) << 1;
		curchr = ((curchr & 0xf0) >> 4) | ((curchr & 0x0f) << 4);
		curchr = ((curchr & 0xcc) >> 2) | ((curchr & 0x33) << 2);
		curchr = ((curchr & 0xaa) >> 1) | ((curchr & 0x55) << 1);
		tstr = translate_alpha(curchr-1);
		if (tstr) {
			int tlen = strlen(tstr);
			if (buffree >= tlen) {
				memcpy(cp, tstr, tlen);
				cp += tlen;
				buffree -= tlen;
			}
		} else if (buffree > 0) {
			*cp++ = curchr-1;
			buffree--;
		}
	}
	*cp = '\0';
	verbprintf(0, "%s\n", buf);
}

/* ---------------------------------------------------------------------- */

static void pocsag_printmessage(struct demod_state *s, struct l2_pocsag_rx *rx, 
				const char *add_name) 
{
	verbprintf(0, "%s%s: Address: %7lu  Function: %1u\n",
		   s->dem_par->name, add_name, rx->adr, rx->func);
	if (!rx->numnibbles)
		return;
	if (service_mask & (0x01 << rx->func)) {
		verbprintf(0, "%s%s: Numeric: ", s->dem_par->name, add_name);
		print_msg_numeric(rx);
	}
	if (service_mask & (0x10 << rx->func)) {
		if (rx->func == 3 && rx->adr >= 4000 &&
		    rx->adr <= 5000) {
			verbprintf(0, "%s%s: Alpha (SKYPER): ", s->dem_par->name, add_name);
			print_msg_skyper(rx);
		} else {
			verbprintf(0, "%s%s: Alpha: ", s->dem_par->name, add_name);
			print_msg_alpha(rx);
		}
	}
}

/* ---------------------------------------------------------------------- */

void pocsag_init(struct demod_state *s)
{
	memset(&s->l2.pocsag, 0, sizeof(s->l2.pocsag));
}

/* ---------------------------------------------------------------------- */

static void do_one_bit(struct demod_state *s, struct l2_pocsag_rx *rx, 
		       unsigned long rx_data, const char *add_name)
{
	unsigned char rxword;

	if (!rx->rx_sync) {
		if (rx_data == POCSAG_SYNC || rx_data == POCSAG_SYNCINFO) {
			rx->rx_sync = 2;
			rx->rx_bit = rx->rx_word = 0;
			rx->func = -1;
			return;
		}
		return;
	}
   	
	if ((++(rx->rx_bit)) < 32) 
		return;
	/*
	 * one complete word received
	 */
	rx->rx_bit = 0;
	/*
	 * check codeword
	 */
	if (pocsag_syndrome(rx_data)) {
		/*
		 * codeword not valid
		 */
		rx->rx_sync--;
		verbprintf(7, "%s: Bad codeword: %08lx%s\n", 
			   s->dem_par->name, rx_data, 
			   rx->rx_sync ? "" : "sync lost");
		if (!(rx->func & (~3))) {
			verbprintf(0, "%s%s: Warning: message garbled\n",
				   s->dem_par->name, add_name);
			pocsag_printmessage(s, rx, add_name);
		}
		rx->func = -1; /* invalidate message */
		return;
	}
	/* do something with the data */
	verbprintf(8, "%s%s: Codeword: %08lx\n", s->dem_par->name, add_name, rx_data);
	rxword = rx->rx_word++;
	if (rxword >= 16) {
		/*
		 * received word shoud be a
		 * frame synch
		 */
		rx->rx_word = 0;
		if ((rx_data == POCSAG_SYNC) ||
		    (rx_data == POCSAG_SYNCINFO))
			rx->rx_sync = 10;
		else
			rx->rx_sync -= 2;
		return;	
	}
	if (rx_data == POCSAG_IDLE) {
		/*
		 * it seems that we can output the message right here
		 */
		if (!(rx->func & (~3)))
			pocsag_printmessage(s, rx, add_name);
		rx->func = -1; /* invalidate message */
		return;
	}
	if (rx_data & 0x80000000) {
		/*
		 * this is a data word
		 */
		unsigned long data;
		unsigned char *bp;
		
		if (rx->func & (~3)) { 
			/*
			 * no message being received 
			 */
			verbprintf(7, "%s%s: Lonesome data codeword: %08lx\n",
				   s->dem_par->name, add_name, rx_data);
			return;
		}
		if (rx->numnibbles > sizeof(rx->buffer)*2 - 5) {
			verbprintf(0, "%s%s: Warning: Message too long\n",
				   s->dem_par->name, add_name);
			pocsag_printmessage(s, rx, add_name);
			rx->func = -1;
			return;
		}
		bp = rx->buffer + (rx->numnibbles >> 1);
		data = rx_data >> 11; 
		if (rx->numnibbles & 1) {
			bp[0] = (bp[0] & 0xf0) | ((data >> 16) & 0xf);
			bp[1] = data >> 8;
			bp[2] = data;
		} else {
			bp[0] = data >> 12;
			bp[1] = data >> 4;
			bp[2] = data << 4;
		}
		rx->numnibbles += 5;
		return;
	}
	/*
	 * process address codeword
	 */
	if (rx_data >= POCSAG_SYNC_WORDS) {
		unsigned char func = (rx_data >> 11) & 3;
		unsigned long adr = ((rx_data >> 10) & 0x1ffff8) | 
			((rxword >> 1) & 7);

		verbprintf(0, "%s%s: Nonstandard address codeword: %08lx "
			   "func %1u adr %08lx\n", s->dem_par->name, add_name, rx_data, 
			   func, adr);
		return;
	}
	if (!(rx->func & (~3))) 
		pocsag_printmessage(s, rx, add_name);
	rx->func = (rx_data >> 11) & 3;
	rx->adr = ((rx_data >> 10) & 0x1ffff8) | ((rxword >> 1) & 7);
	rx->numnibbles = 0;
}

/* ---------------------------------------------------------------------- */

void pocsag_rxbit(struct demod_state *s, int bit)
{
	s->l2.pocsag.rx_data <<= 1;
	s->l2.pocsag.rx_data |= !bit;
	verbprintf(9, " %c ", '1'-(s->l2.pocsag.rx_data & 1));
	do_one_bit(s, s->l2.pocsag.rx, ~(s->l2.pocsag.rx_data), "+");
	do_one_bit(s, s->l2.pocsag.rx+1, s->l2.pocsag.rx_data, "-");
}

/* ---------------------------------------------------------------------- */
