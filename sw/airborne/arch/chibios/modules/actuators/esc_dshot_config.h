#pragma once

#ifndef DSHOT_BIDIR
#define DSHOT_BIDIR FALSE
#endif

/**
 *  By default enable the possibility to mix 16 and 32 bits timer
 */
#ifndef DSHOT_AT_LEAST_ONE_32B_TIMER
#define DSHOT_AT_LEAST_ONE_32B_TIMER TRUE
#endif

#define DSHOT_CHANNELS                4U // depend on the number of channels per timer


/** Base freq of DSHOT signal (in kHz)
 * Possible values are: 150, 300, 600
 */
#ifndef DSHOT_SPEED
#define DSHOT_SPEED 300U
#endif

