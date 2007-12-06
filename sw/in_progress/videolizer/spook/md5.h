#ifndef MD5_H
#define MD5_H

typedef unsigned int u32;
typedef unsigned char u8;

struct MD5Context {
	u32 buf[4];
	u32 bits[2];
	u8 in[64];
};

void MD5Init(struct MD5Context *context);
void MD5Update(struct MD5Context *context, unsigned char const *buf,
	       unsigned len);
void MD5Final(unsigned char digest[16], struct MD5Context *context);
void MD5Transform(u32 buf[4], u32 const in[16]);

typedef struct MD5Context MD5_CTX;

#endif /* MD5_H */
