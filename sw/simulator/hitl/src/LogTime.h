/*
 * Copyright (C) 2015 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file LogTime.h
 *
 * HITL demo version - helper class for time logging and checking
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef LOGTIME_H_
#define LOGTIME_H_

#include <time.h>
#include <stdio.h>
#include <sstream>


class LogTime
{
private:
  const static int secInDay = 86400;

public:
  /**
   * @return current timeval
   */
  static timeval getStart() {
    timeval start_time;
    gettimeofday(&start_time, NULL);
    return start_time;
  }

  /**
   * @param start_t
   * @return time in seconds that elapsed since start_t
   */
  static std::string getTimeSinceStart(timeval start_t) {
    timeval curr_t;
    gettimeofday(&curr_t, NULL);

    double st = (double)start_t.tv_sec + ((double)start_t.tv_usec) / 1000000;
    double ct = (double)curr_t.tv_sec + ((double)curr_t.tv_usec) / 1000000;
    double dt = ct - st;

    std::stringstream stream;
    stream << (double) dt;
    return stream.str();
  }

  /**
   * @param start_t
   * @return time in seconds that elapsed since start_t
   */
  static double getTimeSinceStartDouble(timeval start_t) {
    timeval curr_t;
    gettimeofday(&curr_t, NULL);

    double st = (double)start_t.tv_sec + ((double)start_t.tv_usec) / 1000000;
    double ct = (double)curr_t.tv_sec + ((double)curr_t.tv_usec) / 1000000;
    double dt = ct - st;
    return dt;
  }

  /**
   * return time difference in seconds between d1 and d2
   * @param t1 timeval 1
   * @param t2 timeval 2
   * @return t1-t2
   */
  static double getTimeDiff(timeval t1, timeval t2) {
    double d1 = (double)t1.tv_sec + ((double)t1.tv_usec) / 1e6;
    double d2 = (double)t2.tv_sec + ((double)t2.tv_usec) / 1e6;
    return d1 - d2;
  }

  /**
   * @return GPS TOW
   */
  static uint64_t getTimeOfWeek() {
    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;
    tm t_res;
    localtime_r(&curTime.tv_sec, &t_res);
    const tm *tt = &t_res;

    uint64_t tow = LogTime::secInDay * tt->tm_wday + 3600 * tt->tm_hour + 60 * tt->tm_min + tt->tm_sec; // sec
    tow = tow * 1000; // tow to ms
    tow = tow + milli; // tow with added ms
    tow = tow * 1e6; // tow in nanoseconds

    return tow;
  }

  /**
   * @return current time in Y-m-d H:M:S format
   */
  static std::string get() {
    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;
    char buffer [80];
    // TODO an important note to be considered is that functions like localtime are not thread-safe,
    // and you'd better use localtime_r instead.
    strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", localtime(&curTime.tv_sec));
    return buffer;
  }

  /**
   * @return current time in H:M:S format
   */
  static std::string getTime() {
    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec ; // resolution is us
    char buffer [80];

    tm t_res;
    localtime_r(&curTime.tv_sec, &t_res);
    const tm *tt = &t_res;
    strftime(buffer, 40, "%H:%M:%S", tt);
    char currentTime[84] = "";
    sprintf(currentTime, "%s.%05d", buffer, milli);

    return currentTime;
  }


  /**
   * @return getRawTime timeval
   */
  static std::string getRawTime() {
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    char currentTime[84] = "";
    sprintf(currentTime, "%lld", te.tv_sec * 1000LL + te.tv_usec / 1000);

    return currentTime;
  }

  /**
   * @return time in Y-m-d format
   */
  static std::string getDay() {
    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;

    char buffer [80];

    strftime(buffer, 80, "%Y-%m-%d", localtime(&curTime.tv_sec));

    return buffer;
  }


};

#endif /* LOGTIME_H_ */
