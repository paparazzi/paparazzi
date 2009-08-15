#ifndef CSC_ME4_LINK_H
#define CSC_ME4_LINK_H

#include "std.h"

struct CscMe4Link {
  bool_t    timeout;
  uint32_t  last_msg;
};

extern struct CscMe4Link csc_me4_link;

extern void csc_me4_link_init(void);
extern void csc_me4_link_periodic(void);



#endif /* CSC_ME4_LINK_H */
