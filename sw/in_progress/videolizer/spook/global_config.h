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

/* Don't include this file from any file except conf_parse.c! */

/* Put declarations here for functions that should be called to parse
 * global configuration directives.  All the signatures should be the same
 * so the function can be put into the below array. */

int config_port( int num_tokens, struct token *tokens, void *d );
int config_frameheap( int num_tokens, struct token *tokens, void *d );
int config_rtprange( int num_tokens, struct token *tokens, void *d );
#if 0
int config_sip_proxy( int num_tokens, struct token *tokens, void *d );
int config_sip_name( int num_tokens, struct token *tokens, void *d );
int config_sip_domain( int num_tokens, struct token *tokens, void *d );
int config_sip_username( int num_tokens, struct token *tokens, void *d );
int config_sip_password( int num_tokens, struct token *tokens, void *d );
int config_sip_register( int num_tokens, struct token *tokens, void *d );
#endif

/* The big array describing all the global configuration directives.
 * Declare your function above, then add a new line describing the new
 * directive. */

static struct statement global_statements[] = {
	/* directive name, process function, min args, max args, arg types */

	{ "port", config_port, 1, 1, { TOKEN_NUM } },
	{ "rtprange", config_rtprange, 2, 2, { TOKEN_NUM, TOKEN_NUM } },
	{ "frameheap", config_frameheap, 1, 2, { TOKEN_NUM, TOKEN_NUM } },
#if 0
	{ "sipproxy", config_sip_proxy, 1, 1, { TOKEN_STR } },
	{ "sipname", config_sip_name, 1, 1, { TOKEN_STR } },
	{ "sipdomain", config_sip_domain, 1, 1, { TOKEN_STR } },
	{ "sipusername", config_sip_username, 1, 1, { TOKEN_STR } },
	{ "sippassword", config_sip_password, 1, 1, { TOKEN_STR } },
	{ "sipregister", config_sip_register, 1, 1, { TOKEN_STR } },
#endif

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};
