//
// Created by geart on 5-11-18.
//

#include "log_to_file.h"

// Include standard libraries
#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include "std.h"
#include <string.h>

// Set to default path for bebop if logging path is not set in the airframe config file
#ifndef RL_OBSTACLE_AVOIDANCE_LOG_PATH
#ifndef USE_NPS
    #define RL_OBSTACLE_AVOIDANCE_LOG_PATH /data/ftp/internal_000
#else
    #define RL_OBSTACLE_AVOIDANCE_LOG_PATH /tmp/NPSsimulation
#endif

#endif

// Set pre-processor constants
#define RL_OBSTACLE_AVOIDANCE_PRINT_TO_TERMINAL FALSE


// Declaration of global variables
static FILE *file_logger = NULL; // File pointer
static bool csv_header_written = false;
char rl_obstacle_avoidance_run_filename[50] = "";

/** Start the file logger and open a new file */
void log_to_file_start(char csv_header_line[])
{
    // check if log path exists
    struct stat s;
    int err = stat(STRINGIFY(RL_OBSTACLE_AVOIDANCE_LOG_PATH), &s);

    if(err < 0) {
        // try to make the directory
        printf("Could not find directory: %s \n", STRINGIFY(RL_OBSTACLE_AVOIDANCE_LOG_PATH));
        mkdir(STRINGIFY(RL_OBSTACLE_AVOIDANCE_LOG_PATH), S_IRWXU | S_IRWXG | S_IRWXO);
    }

    // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
    char date_time[80];
    time_t now = time(0);
    struct tm  tstruct;
    tstruct = *localtime(&now);
    strftime(date_time, sizeof(date_time), "%Y-%m-%d_%X", &tstruct);

    uint32_t counter = 0;
    char filename[512];

    // Check for available files
    sprintf(filename, "%s/run_%s.csv", STRINGIFY(RL_OBSTACLE_AVOIDANCE_LOG_PATH), date_time);
    sprintf(rl_obstacle_avoidance_run_filename, "run_%s.csv", date_time);
    while ((file_logger = fopen(filename, "rb"))) {
        fclose(file_logger);

        sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(RL_OBSTACLE_AVOIDANCE_LOG_PATH), date_time, counter);
        sprintf(rl_obstacle_avoidance_run_filename, "%s_%05d.csv", date_time, counter);
        counter++;
    }
    printf("File: %s \n", filename);

    file_logger = fopen(filename, "wb");

    if (file_logger != NULL) {
        fflush(file_logger);
        fprintf(file_logger, "%s", csv_header_line);
        fflush(file_logger);
        csv_header_written = true;
    }

    if (RL_OBSTACLE_AVOIDANCE_PRINT_TO_TERMINAL){
        // For debugging purposes, also print to terminal
        printf("%s",csv_header_line);
    }
}

/** Log one line to the log file **/
void log_to_file_log_line(rl_variable variables[], int array_size){
    if ((file_logger == NULL) || (!csv_header_written)) {
        return;
    }
    // Construct string to be printed
    char line[1000]="";
    int i;
    for (i=0; i < array_size; i++){
        if(i>0){
            strcat(line, ",");
        }
        char temp[40] = "";
        if(strcmp(variables[i].type,"int32_t") == 0){
            sprintf(temp, variables[i].format, *(int32_t *)variables[i].pointer);
        } else if(strcmp(variables[i].type,"uint16_t") == 0){
            sprintf(temp, variables[i].format, *(uint16_t *)variables[i].pointer);
        } else if(strcmp(variables[i].type,"float") == 0){
            sprintf(temp, variables[i].format, *(float *)variables[i].pointer);
        } else if(strcmp(variables[i].type,"long") == 0){
            sprintf(temp, variables[i].format, *(long *)variables[i].pointer);
        } else if(strcmp(variables[i].type,"char32") == 0){
            sprintf(temp, variables[i].format, *(char *)variables[i].pointer);
        } else if(strcmp(variables[i].type,"double") == 0){
            sprintf(temp, variables[i].format, *(double *)variables[i].pointer);
        } else {
            sprintf(temp,"Unknown type:%s",variables[i].type);
        }
        strcat(line, temp);

    }

    if (RL_OBSTACLE_AVOIDANCE_PRINT_TO_TERMINAL){
        // For debugging purposes, also print to terminal
        printf("%s\n", line);
    }

    // Print line to file
    fprintf(file_logger, "%s\n", line);

    // Flush buffer
    fflush(file_logger);

}

/** Stop the logger an nicely close the file */
void log_to_file_stop(void){
    if (file_logger != NULL) {
        fclose(file_logger);
        file_logger = NULL;
        printf("Closed file\n");
    }
    csv_header_written = false;
}

