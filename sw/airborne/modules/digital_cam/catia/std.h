#ifndef STD_H
#define STD_H

#define TRUE (1==1)
#define FALSE (1==0)

#define CameraLinkTransmit(_x) {       \
    uint8_t _tmp = _x;         \
    write(fd,&_tmp,1);         \
  }

#endif
