//
// Created by geart on 5-11-18.
//

#ifndef PAPARAZZI_LOG_TO_FILE_H
#define PAPARAZZI_LOG_TO_FILE_H

#include "modules/rl_obstacle_avoidance/rl_obstacle_avoidance.h"

extern char rl_obstacle_avoidance_run_filename[];

extern void log_to_file_log_line(rl_variable variables[], int array_size);
extern void log_to_file_start(char csv_header_line[]);
extern void log_to_file_stop(void);

#endif //PAPARAZZI_LOG_TO_FILE_H
