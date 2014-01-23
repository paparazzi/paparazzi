#ifndef __RTC_ACCESS_H__
#define __RTC_ACCESS_H__

#include <stdint.h>

void setHour (uint32_t val);
void setMinute (uint32_t val);
void setSecond (uint32_t val);
void setYear (uint32_t val);
void setMonth (uint32_t val);
void setMonthDay (uint32_t val);
void setWeekDay (uint32_t val);
void setRtcFromGps (int16_t week, uint32_t tow);

uint32_t getHour (void);
uint32_t getMinute (void);
uint32_t getSecond (void);
uint32_t getYear (void);
uint32_t getMonth (void);
uint32_t getMonthDay (void);
uint32_t getWeekDay (void);
const char* getWeekDayAscii (void);

#endif //  __RTC_ACCESS_H__
