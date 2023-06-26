#include "stdio.h"
#include "stdlib.h"
#include "circular_buffer.h"
#include <string.h>
#include "tap.h"


uint8_t plop[10] = {1,2,3,4,5,6,7,8,9,10};
uint8_t toto[5] = {105,104,103,102,101};
uint8_t azert[12] = {201,202,203,204,205,206,207,208,209,210,211,212};

uint8_t bubu[20];
struct circular_buffer cbuf;

int main(int argc __attribute_maybe_unused__, char** argv __attribute_maybe_unused__) {

    note("running circular buffer tests");
    plan(15);

    circular_buffer_init(&cbuf, bubu, sizeof(bubu));

    uint8_t bout[18];

    // get on an empty buffer
    int ret = circular_buffer_get(&cbuf, bout, sizeof(bout));
    ok(ret == CIR_ERROR_NO_MSG, "expected CIR_ERROR_NO_MSG, got %d", ret);


    // put plop
    ret = circular_buffer_put(&cbuf, plop, sizeof(plop));
    ok(ret == 0, "expected 0, got %d\n", ret);

    // put toto
    ret = circular_buffer_put(&cbuf, toto, sizeof(toto));
    ok(ret == 0, "expected 0, got %d\n", ret);


    // put azert : should fail
    ret = circular_buffer_put(&cbuf, azert, sizeof(azert));
    ok(ret == CIR_ERROR_NO_SPACE_AVAILABLE, "expected CIR_ERROR_NO_SPACE_AVAILABLE, got %d\n", ret);

    // get first buffer: plop
    ret = circular_buffer_get(&cbuf, bout, sizeof(bout));
    ok(ret == sizeof(plop), "expected %ld, got %d", sizeof(plop), ret);
    ok(memcmp(bout, plop, ret) == 0, "buffer corrupted");


    // put azert. data should wrap around the buffer (8+15>20)
    ret = circular_buffer_put(&cbuf, azert, sizeof(azert));
    ok(ret == 0, "expected 0, got %d\n", ret);

    // get next buffer: toto
    ret = circular_buffer_get(&cbuf, bout, sizeof(bout));
    ok(ret == sizeof(toto), "expected %ld, got %d", sizeof(toto), ret);
    ok(memcmp(bout, toto, ret) == 0, "buffer corrupted");


    // put toto
    ret = circular_buffer_put(&cbuf, toto, sizeof(toto));
    ok(ret == 0, "expected 0, got %d\n", ret);


    // get next buffer: azert
    ret = circular_buffer_get(&cbuf, bout, sizeof(bout));
    ok(ret == sizeof(azert), "expected %ld, got %d", sizeof(azert), ret);
    ok(memcmp(bout, azert, ret) == 0, "buffer corrupted");


    // get next buffer: toto
    ret = circular_buffer_get(&cbuf, bout, sizeof(bout));
    ok(ret == sizeof(toto), "expected %ld, got %d", sizeof(toto), ret);
    ok(memcmp(bout, toto, ret) == 0, "buffer corrupted");


    // get on an empty buffer
    ret = circular_buffer_get(&cbuf, bout, sizeof(bout));
    ok(ret == CIR_ERROR_NO_MSG, "expected CIR_ERROR_NO_MSG, got %d", ret);

    return 0;
}
