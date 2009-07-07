#ifndef BOOZ_RADIO_CONTROL_SPEKTRUM_DX7SE_H
#define BOOZ_RADIO_CONTROL_SPEKTRUM_DX7SE_H

#define RADIO_CONTROL_NB_CHANNEL 7
#define RADIO_CONTROL_ROLL       0
#define RADIO_CONTROL_THROTTLE   1
#define RADIO_CONTROL_PITCH      2
#define RADIO_CONTROL_YAW        3
#define RADIO_CONTROL_MODE       5

#define RC_SPK_SYNC_2 0x12

#define RC_SPK_THROWS { MAX_PPRZ/MAX_SPK, \
                        MAX_PPRZ/MAX_SPK, \
                       -MAX_PPRZ/MAX_SPK, \
                        MAX_PPRZ/MAX_SPK, \
                        MAX_PPRZ/MAX_SPK, \
                        MAX_PPRZ/MAX_SPK, \
                        MAX_PPRZ/MAX_SPK }



#endif /* BOOZ_RADIO_CONTROL_SPEKTRUM_DX7SE_H */
