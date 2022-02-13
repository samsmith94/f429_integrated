// #include "RTC.h"

// STM32RTC &rtc = STM32RTC::getInstance();

// void Internal_RTC_init()
// {
//     rtc.setClockSource(STM32RTC::LSE_CLOCK);
//     rtc.begin(); // initialize RTC 24H format
// }

// void Internal_RTC_setTime(Internal_RTC_Time time)
// {
//     rtc.setTime(time.hour, time.min, time.sec);
// }

// void Internal_RTC_setDate(Internal_RTC_Date date)
// {
//     uint8_t weekDay = dayofweek(date.year, date.month, date.day);
//     rtc.setDate(weekDay, date.day, date.month, date.year);
// }

// void Internal_RTC_getTime(Internal_RTC_Time *time)
// {
//     time->hour = rtc.getHours();
//     time->min = rtc.getMinutes();
//     time->sec = rtc.getSeconds();
// }

// void Internal_RTC_getDate(Internal_RTC_Date *date)
// {
//     date->year = rtc.getYear();
//     date->month = rtc.getMonth();
//     date->day = rtc.getDay();
// }

// void Internal_RTC_printDateTime(Internal_RTC_Date date, Internal_RTC_Time time)
// {
//     // Print date...
//     Serial.printf("%02d/%02d/%02d ", date.year, date.month, date.day);
//     // ...and time
//     Serial.printf("%02d:%02d:%02d\n", time.hour, time.min, time.sec);
// }

// const int dayofweek(int year, int month, int day) {
//   /* using C99 compound literals in a single line: notice the splicing */
//   return ((const int [])                                         \
//           {1, 2, 3, 4, 5, 6, 0})[           \
//       (                                                            \
//           day                                                      \
//         + ((153 * (month + 12 * ((14 - month) / 12) - 3) + 2) / 5) \
//         + (365 * (year + 4800 - ((14 - month) / 12)))              \
//         + ((year + 4800 - ((14 - month) / 12)) / 4)                \
//         - ((year + 4800 - ((14 - month) / 12)) / 100)              \
//         + ((year + 4800 - ((14 - month) / 12)) / 400)              \
//         - 32045                                                    \
//       ) % 7];
// }