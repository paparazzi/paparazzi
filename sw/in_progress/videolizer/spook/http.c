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
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <pmsg.h>
#include <rtp.h>
#include <conf_parse.h>

void write_access_log( char *path, struct sockaddr *addr, int code, char *req,
		int length, char *referer, char *user_agent );

struct http_stream_session;

struct http_location {
	struct loc_node node;
	struct stream_destination *input;
	struct frame *frame;
	int length_with_jfif;
	struct http_stream_session *sess_list;
	int streaming;
};

struct http_stream_session {
	struct http_stream_session *next;
	struct http_stream_session *prev;
	struct http_location *loc;
	struct conn *conn;
};

static struct http_location *http_loc_list = NULL;

static struct loc_node *node_find_location( struct loc_node *list,
						char *path, int len )
{
	struct loc_node *loc;

	for( loc = list; loc; loc = loc->next )
		if( ! strncmp( path, loc->path, strlen( loc->path ) )
				&& ( loc->path[len] == '/'
					|| loc->path[len] == 0 ) )
			return loc;
	return NULL;
}

static struct http_location *find_http_location( char *path )
{
	int len;

	len = strlen( path );
	if( path[len - 1] == '/' ) --len;

	return (struct http_location *)node_find_location(
			(struct loc_node *)http_loc_list, path, len );
}

static void init_location( struct loc_node *node, char *path,
				struct loc_node **list )
{
	node->next = *list;
	node->prev = NULL;
	if( node->next ) node->next->prev = node;
	*list = node;
	strcpy( node->path, path );
}

static void log_request( struct req *req, int code, int length )
{
	char *ref, *ua;

	ref = get_header( req->req, "referer" );
	ua = get_header( req->req, "user-agent" );
	write_access_log( NULL, (struct sockaddr *)&req->conn->client_addr,
		code, req->conn->req_buf, length,
		ref ? ref : "-", ua ? ua : "-" );
}

static void http_create_reply( struct req *req, int code, char *reply )
{
	req->resp = new_pmsg( 512 );
	req->resp->type = PMSG_RESP;
	req->resp->proto_id = add_pmsg_string( req->resp, "HTTP/1.0" );
	req->resp->sl.stat.code = code;
	req->resp->sl.stat.reason = add_pmsg_string( req->resp, reply );
}

static void http_send_error( struct req *req, int code, char *reply, char *t )
{
	log_request( req, code, 0 );
	http_create_reply( req, code, reply );
	add_header( req->resp, "Connection", "close" );
	add_header( req->resp, "Content-Type", "text/html" );
	req->conn->drop_after = 1;
	if( tcp_send_pmsg( req->conn, req->resp, strlen( t ) ) >= 0 )
		send_data( req->conn, t, strlen( t ) );
}

void http_conn_disconnect( struct conn *c )
{
	struct http_stream_session *s =
				(struct http_stream_session *)c->proto_state;

	if( s->next ) s->next->prev = s->prev;
	if( s->prev ) s->prev->next = s->next;
	else s->loc->sess_list = s->next;
	free( s );
}

static void add_http_stream_session( struct http_location *loc, struct conn *c )
{
	struct http_stream_session *s;

	s = (struct http_stream_session *)
		malloc( sizeof( struct http_stream_session ) );
	s->next = loc->sess_list;
	s->prev = NULL;
	if( s->next ) s->next->prev = s;
	loc->sess_list = s;
	s->loc = loc;
	s->conn = c;
	c->proto_state = s;
}

static void send_frame_with_jfif( struct http_location *loc, struct conn *c )
{
	static unsigned char add_jfif[] =
		{ 0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46,
		  0x00, 0x01, 0x02, 0x01, 0x00, 0x48, 0x00, 0x48, 0x00, 0x00 };

	if( loc->length_with_jfif > loc->frame->length )
	{
		send_data( c, add_jfif, 20 );
		send_data( c, loc->frame->d + 2, loc->frame->length - 2 );
	} else
		send_data( c, loc->frame->d, loc->frame->length );
}

static void send_multipart_frame( struct http_location *loc, struct conn *c )
{
	char hdrs[128];
	int hlen;

	hlen = sprintf( hdrs, "--boundary\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", loc->length_with_jfif );
	if( avail_send_buf( c ) < loc->length_with_jfif + hlen ) return;
	send_data( c, hdrs, hlen );
	send_frame_with_jfif( loc, c );
}

static int handle_GET( struct req *req )
{
	struct http_location *loc;

	loc = find_http_location( req->req->sl.req.uri );
	if( ! loc || strlen( req->req->sl.req.uri ) > strlen( loc->node.path ) )
	{
		http_send_error( req, 404, "Not Found", "<html><body><p>File not found</p></body></html>" );
		return 0;
	}
	if( ! loc->frame ||
			avail_send_buf( req->conn ) < loc->frame->length + 400 )
	{
		http_send_error( req, 404, "Not Found", "<html><body><p>Image not available</p></body></html>" );
		return 0;
	}
	http_create_reply( req, 200, "OK" );
	add_header( req->resp, "Expires", "0" );
	add_header( req->resp, "Pragma", "no-cache" );
	add_header( req->resp, "Cache-Control", "no-cache" );
	if( loc->streaming )
	{
		log_request( req, 200, 0 );
		add_header( req->resp, "Content-Type",
			"multipart/x-mixed-replace;boundary=\"boundary\"" );
		tcp_send_pmsg( req->conn, req->resp, -1 );
		add_http_stream_session( loc, req->conn );
		send_multipart_frame( loc, req->conn );
	} else
	{
		log_request( req, 200, loc->frame->length );
		add_header( req->resp, "Connection", "close" );
		add_header( req->resp, "Content-Type", "image/jpeg" );
		req->conn->drop_after = 1;
		if( tcp_send_pmsg( req->conn, req->resp,
					loc->length_with_jfif ) >= 0 )
			send_frame_with_jfif( loc, req->conn );
	}
	//send_data( req->conn, rep, sprintf( rep, "HTTP/1.0 200 OK\r\nConnection: close\r\nContent-Type: multipart/x-mixed-replace;boundary=\"boundary\"\r\n\r\n--boundary\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", loc->frame->length ) );
	//send_data( req->conn, loc->frame->d, loc->frame->length );
	return 0;
}

static int handle_unknown( struct req *req )
{
	http_send_error( req, 501, "Not Implemented", "<html><body><p>POST requests are unsupported here</p></body></html>" );
	return 0;
}

int http_handle_msg( struct req *req )
{
	int ret;

	if( ! strcasecmp( req->req->sl.req.method, "GET" ) )
		ret = handle_GET( req );
	else ret = handle_unknown( req );
	return ret;
}

static void jpeg_next_frame( struct frame *f, void *d )
{
	struct http_location *loc = (struct http_location *)d;
	struct http_stream_session *s;

	if( f->d[0] != 0xff || f->d[1] != 0xd8 || f->d[2] != 0xff )
	{
		spook_log( SL_WARN, "http: received a malformed JPEG frame" );
		unref_frame( f );
		return;
	}
	if( loc->frame ) unref_frame( loc->frame );
	loc->frame = f;
	loc->length_with_jfif = f->length;
	if( f->d[3] != 0xe0 || f->d[10] != 0 || strcmp( f->d + 6, "JFIF" ) )
		loc->length_with_jfif += 18;
	for( s = loc->sess_list; s; s = s->next )
		send_multipart_frame( loc, s->conn );
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct http_location *loc;

	loc = (struct http_location *)malloc( sizeof( struct http_location ) );
	init_location( (struct loc_node *)loc, "",
			(struct loc_node **)&http_loc_list );
	loc->input = NULL;
	loc->frame = NULL;
	loc->sess_list = NULL;
	loc->streaming = 0;

	return loc;
}

static int end_block( void *d )
{
	struct http_location *loc = (struct http_location *)d;

	if( ! loc->input )
	{
		spook_log( SL_ERR, "http output: missing input stream name" );
		return -1;
	}
	if( ! *loc->node.path )
	{
		spook_log( SL_ERR, "http output: missing URL path" );
		return -1;
	}
	set_waiting( loc->input, 1 );

	return 0;
}

static int set_input( int num_tokens, struct token *tokens, void *d )
{
	struct http_location *loc = (struct http_location *)d;
	int format = FORMAT_JPEG;

	if( ! ( loc->input = connect_to_stream( tokens[1].v.str,
					jpeg_next_frame, loc, &format, 1 ) ) )
	{
		spook_log( SL_ERR,
			"http output: unable to connect to stream \"%s\"",
			tokens[1].v.str );
		return -1;
	}
	return 0;
}

static int set_path( int num_tokens, struct token *tokens, void *d )
{
	struct http_location *loc = (struct http_location *)d;

	strcpy( loc->node.path, tokens[1].v.str );
	return 0;
}

static int set_mode( int num_tokens, struct token *tokens, void *d )
{
	struct http_location *loc = (struct http_location *)d;

	if( ! strcasecmp( tokens[1].v.str, "single" ) )
		loc->streaming = 0;
	else if( ! strcasecmp( tokens[1].v.str, "stream" ) )
		loc->streaming = 1;
	else
	{
		spook_log( SL_ERR,
			"http output: Mode must be 'single' or 'stream'" );
		return -1;
	}
	return 0;
}

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "input", set_input, 1, 1, { TOKEN_STR } },
	{ "path", set_path, 1, 1, { TOKEN_STR } },
	{ "mode", set_mode, 1, 1, { TOKEN_STR } },

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

int http_init(void)
{
	register_config_context( "output", "http", start_block, end_block,
					config_statements );
	return 0;
}
