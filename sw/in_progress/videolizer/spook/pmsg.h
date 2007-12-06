struct hdrf {
	char *name;
	char *value;
};

#define MAX_FIELDS	32

struct pmsg {
	unsigned char *msg;
	int max_len;
	int msg_len;
	struct hdrf fields[MAX_FIELDS];
	int header_count;
	enum { PMSG_REQ, PMSG_RESP } type;
	char *proto_id;
	union {
		struct {
			char *method;
			char *uri;
		} req;
		struct {
			int code;
			char *reason;
		} stat;
	} sl;
};

char *add_pmsg_string( struct pmsg *msg, char *s );
int parse_pmsg( struct pmsg *msg );
char *get_header( struct pmsg *msg, char *name );
int add_header( struct pmsg *msg, char *name, char *value );
int add_header_printf( struct pmsg *msg, char *name, char *fmt, ... );
int replace_header( struct pmsg *msg, char *name, char *value );
int copy_headers( struct pmsg *dest, struct pmsg *src, char *name );
int get_param( char *value, char *tag, char *dest, int size );
struct pmsg *new_pmsg( int size );
void free_pmsg( struct pmsg *msg );

/* http-auth.c */
int check_digest_response( struct pmsg *msg, char *realm,
				char *username, char *password );
int add_digest_challenge( struct pmsg *msg, char *realm, int stale );
