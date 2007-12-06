/*
 * Copyright (C) 2004 Nathan Lutchansky <lutchann@litech.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <rtp.h>
#include <conf_parse.h>

static int rtp_port_start = 50000, rtp_port_end = 60000;

static int rtcp_send( struct rtp_endpoint *ep );

static void rtcp_fire( struct event_info *ei, void *d )
{
	struct rtp_endpoint *ep = (struct rtp_endpoint *)d;

	rtcp_send( ep );
}

struct rtp_endpoint *new_rtp_endpoint( int payload )
{
	struct rtp_endpoint *ep;

	if( ! ( ep = (struct rtp_endpoint *)
			malloc( sizeof( struct rtp_endpoint ) ) ) )
		return NULL;
	ep->payload = payload;
	ep->max_data_size = 8192; /* default maximum */
	ep->ssrc = 0;
	random_bytes( (unsigned char *)&ep->ssrc, 4 );
	ep->start_timestamp = 0;
	random_bytes( (unsigned char *)&ep->start_timestamp, 4 );
	ep->last_timestamp = ep->start_timestamp;
	ep->seqnum = 0;
	random_bytes( (unsigned char *)&ep->seqnum, 2 );
	ep->packet_count = 0;
	ep->octet_count = 0;
	ep->rtcp_send_event = add_timer_event( 5000, 0, rtcp_fire, ep );
	set_event_enabled( ep->rtcp_send_event, 0 );
	ep->force_rtcp = 1;
	gettimeofday( &ep->last_rtcp_recv, NULL );
	ep->trans_type = 0;

	return ep;
}

void del_rtp_endpoint( struct rtp_endpoint *ep )
{
	remove_event( ep->rtcp_send_event );

	switch( ep->trans_type )
	{
	case RTP_TRANS_UDP:
		remove_event( ep->trans.udp.rtp_event );
		close( ep->trans.udp.rtp_fd );
		remove_event( ep->trans.udp.rtcp_event );
		close( ep->trans.udp.rtcp_fd );
		break;
	case RTP_TRANS_INTER:
		interleave_disconnect( ep->trans.inter.conn,
						ep->trans.inter.rtp_chan );
		interleave_disconnect( ep->trans.inter.conn,
						ep->trans.inter.rtcp_chan );
		break;
	}

	free( ep );
}

void update_rtp_timestamp( struct rtp_endpoint *ep, int time_increment )
{
	ep->last_timestamp += time_increment;
	ep->last_timestamp &= 0xFFFFFFFF;
}

static void udp_rtp_read( struct event_info *ei, void *d )
{
	struct rtp_endpoint *ep = (struct rtp_endpoint *)d;
	unsigned char buf[16384];
	int ret;

	ret = read( ep->trans.udp.rtp_fd, buf, sizeof( buf ) );
	if( ret > 0 )
	{
		/* some SIP phones don't send RTCP */
		gettimeofday( &ep->last_rtcp_recv, NULL );
		return;
	} else if( ret < 0 )
		spook_log( SL_VERBOSE, "error on UDP RTP socket: %s",
			strerror( errno ) );
	else spook_log( SL_VERBOSE, "UDP RTP socket closed" );
	ep->session->teardown( ep->session, ep );
}

static void udp_rtcp_read( struct event_info *ei, void *d )
{
	struct rtp_endpoint *ep = (struct rtp_endpoint *)d;
	unsigned char buf[16384];
	int ret;

	ret = read( ep->trans.udp.rtcp_fd, buf, sizeof( buf ) );
	if( ret > 0 )
	{
		spook_log( SL_DEBUG, "received RTCP packet from client" );
		gettimeofday( &ep->last_rtcp_recv, NULL );
		return;
	} else if( ret < 0 )
		spook_log( SL_VERBOSE, "error on UDP RTCP socket: %s",
			strerror( errno ) );
	else spook_log( SL_VERBOSE, "UDP RTCP socket closed" );
	ep->session->teardown( ep->session, ep );
}

void interleave_recv_rtcp( struct rtp_endpoint *ep, unsigned char *d, int len )
{
	spook_log( SL_DEBUG, "received RTCP packet from client" );
	gettimeofday( &ep->last_rtcp_recv, NULL );
}

static int rtcp_send( struct rtp_endpoint *ep )
{
	struct timeval now;
	unsigned char buf[16384];
	unsigned int ntp_sec, ntp_usec;
	struct iovec v[1];

	gettimeofday( &now, NULL );

	ep->force_rtcp = 0;
//	spook_log( SL_DEBUG, "sending RTCP packet" );

	/* Grrr...  QuickTime apparently doesn't send RTCP over TCP */
	if( ep->trans_type != RTP_TRANS_INTER )
	{
		if( now.tv_sec - ep->last_rtcp_recv.tv_sec > 20 )
		{
			spook_log( SL_VERBOSE, "client timeout (no RTCP received in 20 seconds)" );
			ep->session->teardown( ep->session, ep );
			return -1;
		}
	}

	ntp_sec = now.tv_sec + 0x83AA7E80;
	ntp_usec = (double)( (double)now.tv_usec * (double)0x4000000 ) / 15625.0;
	
	//spook_log( SL_DEBUG, "ssrc=%u, ntp_sec=%u, ntp_usec=%u last_timestamp=%u packet_count=%d octet_count=%d",
	//		x->ssrc, ntp_sec, ntp_usec, x->last_timestamp, x->packet_count, x->octet_count );

	buf[0] = 2 << 6; // version
	buf[1] = 200; // packet type is Sender Report
	PUT_16( buf + 2, 6 ); // length in words minus one
	PUT_32( buf + 4, ep->ssrc );
	PUT_32( buf + 8, ntp_sec );
	PUT_32( buf + 12, ntp_usec );
	PUT_32( buf + 16, ep->last_timestamp );
	PUT_32( buf + 20, ep->packet_count );
	PUT_32( buf + 24, ep->octet_count );
	buf[28] = ( 2 << 6 ) | 1; // version; source count = 1
	buf[29] = 202; // packet type is Source Description
	PUT_16( buf + 30, 4 ); // length in words minus one
	PUT_32( buf + 32, ep->ssrc );
	buf[36] = 0x01; // field type is CNAME
	buf[37] = 14; // text length
	memcpy( buf + 38, "Unnamed stream", 14 );
	switch( ep->trans_type )
	{
	case RTP_TRANS_UDP:
		if( send( ep->trans.udp.rtcp_fd, buf, 52, 0 ) < 0 )
			spook_log( SL_VERBOSE, "error sending UDP RTCP frame: %s",
					strerror( errno ) );
		else return 0;
		break;
	case RTP_TRANS_INTER:
		v[0].iov_base = buf;
		v[0].iov_len = 52;
		if( interleave_send( ep->trans.inter.conn,
				ep->trans.inter.rtcp_chan, v, 1 ) < 0 )
			spook_log( SL_VERBOSE, "error sending interleaved RTCP frame" );
		else return 0;
		break;
	}
	ep->session->teardown( ep->session, ep );
	return -1;
}

int connect_udp_endpoint( struct rtp_endpoint *ep,
		struct in_addr dest_ip, int dest_port, int *our_port )
{
	struct sockaddr_in rtpaddr, rtcpaddr;
	int port, success = 0, i, max_tries, rtpfd = -1, rtcpfd = -1;

	rtpaddr.sin_family = rtcpaddr.sin_family = AF_INET;
	rtpaddr.sin_addr.s_addr = rtcpaddr.sin_addr.s_addr = 0;

	port = rtp_port_start + random() % ( rtp_port_end - rtp_port_start );
	if( port & 0x1 ) ++port;
	max_tries = ( rtp_port_end - rtp_port_start + 1 ) / 2;

	for( i = 0; i < max_tries; ++i )
	{
		if( port + 1 > rtp_port_end ) port = rtp_port_start;
		rtpaddr.sin_port = htons( port );
		rtcpaddr.sin_port = htons( port + 1 );
		if( rtpfd < 0 &&
			( rtpfd = socket( PF_INET, SOCK_DGRAM, 0 ) ) < 0 )
		{
			spook_log( SL_WARN, "unable to create UDP RTP socket: %s",
					strerror( errno ) );
			return -1;
		}
		if( rtcpfd < 0 &&
			( rtcpfd = socket( PF_INET, SOCK_DGRAM, 0 ) ) < 0 )
		{
			spook_log( SL_WARN, "unable to create UDP RTCP socket: %s",
					strerror( errno ) );
			close( rtpfd );
			return -1;
		}
		if( bind( rtpfd, (struct sockaddr *)&rtpaddr,
					sizeof( rtpaddr ) ) < 0 )
		{
			if( errno == EADDRINUSE )
			{
				port += 2;
				continue;
			}
			spook_log( SL_WARN, "strange error when binding RTP socket: %s",
					strerror( errno ) );
			close( rtpfd );
			close( rtcpfd );
			return -1;
		}
		if( bind( rtcpfd, (struct sockaddr *)&rtcpaddr,
					sizeof( rtcpaddr ) ) < 0 )
		{
			if( errno == EADDRINUSE )
			{
				close( rtpfd );
				rtpfd = -1;
				port += 2;
				continue;
			}
			spook_log( SL_WARN, "strange error when binding RTCP socket: %s",
					strerror( errno ) );
			close( rtpfd );
			close( rtcpfd );
			return -1;
		}
		success = 1;
		break;
	}
	if( ! success )
	{
		spook_log( SL_WARN, "ran out of UDP RTP ports!" );
		return -1;
	}
	rtpaddr.sin_family = rtcpaddr.sin_family = AF_INET;
	rtpaddr.sin_addr = rtcpaddr.sin_addr = dest_ip;
	rtpaddr.sin_port = htons( dest_port );
	rtcpaddr.sin_port = htons( dest_port + 1 );
	if( connect( rtpfd, (struct sockaddr *)&rtpaddr,
				sizeof( rtpaddr ) ) < 0 )
	{
		spook_log( SL_WARN, "strange error when connecting RTP socket: %s",
				strerror( errno ) );
		close( rtpfd );
		close( rtcpfd );
		return -1;
	}
	if( connect( rtcpfd, (struct sockaddr *)&rtcpaddr,
				sizeof( rtcpaddr ) ) < 0 )
	{
		spook_log( SL_WARN, "strange error when connecting RTCP socket: %s",
				strerror( errno ) );
		close( rtpfd );
		close( rtcpfd );
		return -1;
	}
	i = sizeof( rtpaddr );
	if( getsockname( rtpfd, (struct sockaddr *)&rtpaddr, &i ) < 0 )
	{
		spook_log( SL_WARN, "strange error from getsockname: %s",
				strerror( errno ) );
		close( rtpfd );
		close( rtcpfd );
		return -1;
	}

	ep->max_data_size = 1400; /* good guess for preventing fragmentation */
	ep->trans_type = RTP_TRANS_UDP;
	sprintf( ep->trans.udp.sdp_addr, "IP4 %s",
				inet_ntoa( rtpaddr.sin_addr ) );
	ep->trans.udp.sdp_port = ntohs( rtpaddr.sin_port );
	ep->trans.udp.rtp_fd = rtpfd;
	ep->trans.udp.rtcp_fd = rtcpfd;
	ep->trans.udp.rtp_event = add_fd_event( rtpfd, 0, 0, udp_rtp_read, ep );
	ep->trans.udp.rtcp_event =
				add_fd_event( rtcpfd, 0, 0, udp_rtcp_read, ep );

	*our_port = port;

	return 0;
}

void connect_interleaved_endpoint( struct rtp_endpoint *ep,
		struct conn *conn, int rtp_chan, int rtcp_chan )
{
	ep->trans_type = RTP_TRANS_INTER;
	ep->trans.inter.conn = conn;
	ep->trans.inter.rtp_chan = rtp_chan;
	ep->trans.inter.rtcp_chan = rtcp_chan;
}

int send_rtp_packet( struct rtp_endpoint *ep, struct iovec *v, int count,
			unsigned int timestamp, int marker )
{
	unsigned char rtphdr[12];
	struct msghdr mh;
	int i;

	ep->last_timestamp = ( ep->start_timestamp + timestamp )
					& 0xFFFFFFFF;

//	spook_log( SL_DEBUG, "RTP: payload %d, seq %u, time %u, marker %d",
//		ep->payload, ep->seqnum, ep->last_timestamp, marker );

	rtphdr[0] = 2 << 6; /* version */
	rtphdr[1] = ep->payload;
	if( marker ) rtphdr[1] |= 0x80;
	PUT_16( rtphdr + 2, ep->seqnum );
	PUT_32( rtphdr + 4, ep->last_timestamp );
	PUT_32( rtphdr + 8, ep->ssrc );

	v[0].iov_base = rtphdr;
	v[0].iov_len = 12;

	switch( ep->trans_type )
	{
	case RTP_TRANS_UDP:
		memset( &mh, 0, sizeof( mh ) );
		mh.msg_iov = v;
		mh.msg_iovlen = count;
		if( sendmsg( ep->trans.udp.rtp_fd, &mh, 0 ) < 0 )
		{
			spook_log( SL_VERBOSE, "error sending UDP RTP frame: %s",
					strerror( errno ) );
			ep->session->teardown( ep->session, ep );
			return -1;
		}
		break;
	case RTP_TRANS_INTER:
		if( interleave_send( ep->trans.inter.conn,
				ep->trans.inter.rtp_chan, v, count ) < 0 )
		{
			spook_log( SL_VERBOSE, "error sending interleaved RTP frame" );
			ep->session->teardown( ep->session, ep );
			return -1;
		}
		break;
	}

	for( i = 0; i < count; ++i ) ep->octet_count += v[i].iov_len;
	++ep->packet_count;

	if( ep->force_rtcp )
	{
		if( rtcp_send( ep ) < 0 ) return -1;
		set_event_enabled( ep->rtcp_send_event, 1 );
	}

	ep->seqnum = ( ep->seqnum + 1 ) & 0xFFFF;

	return 0;
}

/********************* GLOBAL CONFIGURATION DIRECTIVES ********************/

int config_rtprange( int num_tokens, struct token *tokens, void *d )
{
	rtp_port_start = tokens[1].v.num;
	rtp_port_end = tokens[2].v.num;

	if( rtp_port_start & 0x1 ) ++rtp_port_start;
	if( ! ( rtp_port_end & 0x1 ) ) --rtp_port_end;

	spook_log( SL_DEBUG, "RTP port range is %d-%d",
				rtp_port_start, rtp_port_end );

	if( rtp_port_end - rtp_port_start + 1 < 8 )
	{
		spook_log( SL_ERR, "at least 8 ports are needed for RTP" );
		exit( 1 );
	}

	return 0;
}
