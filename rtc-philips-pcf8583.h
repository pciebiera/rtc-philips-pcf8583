#ifndef _rtc_philips_pcf8583
#define _rtc_philips_pcf8583

#define RTC_Address                         0b10100000
#define RTC_I2C_Read                        0x01
#define RTC_I2C_Write                       0x00
#define RTC_ADDR_CONFIG                     0x00
#define RTC_ADDR_ALARM_CONFIG               0x08
#define RTC_ALARM_MODE_DATED                0x01
#define RTC_ALARM_MODE_TIMED                0x02
#define RTC_ALARM_INTERRUPT                 0x01
#define RTC_ALARM_INTERRUPT_NONE            0x00

#define RTC_ALARM_TIMED_UNIT_1_100_SECONDS  0b00000001
#define RTC_ALARM_TIMER_UNIT_SECONDS        0b00000010
#define RTC_ALARM_TIMED_UNIT_MINUTES        0b00000011
#define RTC_ALARM_TIMED_UNIT_HOURS          0b00000100
#define RTC_ALARM_TIMED_UNIT_DAYS           0b00000101

#define RTC_ADDR_ALARM_1_100_SECONDS        0x09
#define RTC_ADDR_ALARM_SECONDS              0x0A
#define RTC_ADDR_ALARM_MINUTES              0x0B
#define RTC_ADDR_ALARM_HOURS                0x0C
#define RTC_ADDR_ALARM_DATE                 0x0D
#define RTC_ADDR_ALARM_MONTH                0x0E
#define RTC_ADDR_ALARM_TIMER                0x0F

#define RTC_ADDR_TIME_1_100_SECONDS         0x01
#define RTC_ADDR_TIME_SECONDS               0x02
#define RTC_ADDR_TIME_MINUTES               0x03
#define RTC_ADDR_TIME_HOURS                 0x04
#define RTC_ADDR_TIME_YEAR_DATE             0x05
#define RTC_ADDR_TIME_WEEKDAYS_MONTH        0x06
#define RTC_ADDR_TIMER                      0x07

#define RTC_OVERWRITE                       0x09
#define RTC_PAUSE_CLOCK                     0x01
#define RTC_DONT_PAUSE_CLOCK                0x00

void rtcStartClock ( void );
void rtcStopClock ( void );

void rtcSetHour( unsigned char );
void rtcSetMinute( unsigned char );
void rtcSetSecond( unsigned char );
void rtcSetHundrethsSeconds ( unsigned char );
void rtcSetMonth ( unsigned char );
void rtcSetDate ( unsigned char );
void rtcSetYear ( unsigned int );

void rtcAlarmEnable ( unsigned char, unsigned char, unsigned char );
void rtcSetAlarmInterval ( unsigned char );
void rtcAlarmDisable ( void );

void rtcSetAlarmHour ( unsigned char );
void rtcSetAlarmMinute ( unsigned char );
void rtcSetAlarmSecond ( unsigned char );
void rtcSetAlarmHundrethsSeconds ( unsigned char );
void rtcSetAlarmMonth ( unsigned char );
void rtcSetAlarmDate ( unsigned char );

void rtcAlarmReset ( void );
void rtcTimerAlarmReset ( void );

struct Timestamp rtcGetTimestamp ( unsigned char );

struct Timestamp {
    unsigned char Month;
    unsigned char Date;
    unsigned int Year;
    unsigned char Hour;
    unsigned char Minute;
    unsigned char Second;
};

#include <rtc-philips-pcf8583.c>

#endif
