#include "circular_buffer.h"
#include <string.h>


/*
 * @param buf: buffer to copy data to
 * len: length of the buffer
 * return number of bytes written in the buffer
*/
int cir_buf_get(struct cir_buf* rg, uint8_t* buf, size_t len) {
	// buffer empty
	if(rg->read_offset == rg->write_offset) { return CIR_ERROR_NO_MSG; }
	// LEN| MSG...| LEN | MSG...
	uint8_t msg_len = rg->_buf[rg->read_offset];
	// output buffer too small
	if(len < msg_len) { return CIR_ERROR_BUFFER_TOO_SMALL; }

	size_t end_offset = rg->read_offset + msg_len + 1;
	if(end_offset >= rg->_buf_len) {
		end_offset -= rg->_buf_len;
	}
	uint8_t* start = rg->_buf + rg->read_offset + 1;

	if(end_offset > rg->read_offset+1){	
		memcpy(buf, start, msg_len);
	}
	else {
		size_t len1 = rg->_buf_len - (rg->read_offset + 1);
		size_t len2 = len - len1;
		memcpy(buf, start, len1);
		memcpy(buf+len1, rg->_buf, len2);
	}
	
	int nb_bytes = msg_len;
	rg->read_offset = end_offset;
	return nb_bytes;
}

int cir_buf_put(struct cir_buf* rg, uint8_t* buf, size_t len) {
	if((len + 3) > rg->_buf_len)
	{
		return CIR_MSG_TOO_LARGE;
	}
	size_t end_offset = rg->write_offset + len + 1;
	if(end_offset >= rg->_buf_len){
		end_offset -= rg->_buf_len;
	} 
	if(rg->read_offset < rg->write_offset) {
		if(end_offset > rg->read_offset && end_offset < rg->write_offset) { 
			return CIR_ERROR_NO_SPACE_AVAILABLE;
		}
	}
	if(rg->read_offset > rg->write_offset) {
		if(end_offset > rg->read_offset || end_offset < rg->write_offset) {
			return CIR_ERROR_NO_SPACE_AVAILABLE;
		}
	}

	rg->_buf[rg->write_offset] = len;
	if(end_offset > rg->write_offset){
		memcpy(rg->_buf+rg->write_offset+1, buf, len);
	}
	else {
		size_t len1 = rg->_buf_len-(rg->write_offset+1);
		size_t len2 = len - len1;
		memcpy(rg->_buf+rg->write_offset+1, buf, len1);
		memcpy(rg->_buf, buf+len1, len2);
	}
	
	rg->write_offset = end_offset;
	return 0;
}

void cir_buf_init(struct cir_buf* rg, uint8_t* buffer, size_t len) {
	rg->_buf = buffer;
	rg->_buf_len = len;
	rg->read_offset = 0;
	rg->write_offset = 0;
}
