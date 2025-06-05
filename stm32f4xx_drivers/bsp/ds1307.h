/*
 * ds1307.h
 *
 *  Created on: Jun 5, 2025
 *      Author: hugo-juarez
 */

#ifndef DS1307_H_
#define DS1307_H_

/******************************************
 *              CONFIGURATION
 ******************************************/

// --- Address ---
#define DS1307_I2C_ADDR				0x68

// --- Register Addresses ---
#define DS1307_ADDR_SEC				0x00
#define DS1307_ADDR_MIN				0x01
#define DS1307_ADDR_HRS				0x02
#define DS1307_ADDR_DAY				0x03
#define DS1307_ADDR_DATE			0x04
#define DS1307_ADDR_MONTH			0x05
#define DS1307_ADDR_YEAR			0x06

// --- Time Format ---
#define TIME_FORMAT_12HRS_AM		0
#define TIME_FORMAT_12HRS_PM		1
#define TIME_FORMAT_24HRS			2

// --- Days ---
#define SUNDAY						1
#define MONDAY						2
#define TUESDAY						3
#define WEDNESDAY					4
#define THURSDAY					5
#define FRIDAY						6
#define SATURDAY					7

// --- Date Struct ---
typedef struct {
	uint8_t data;
	uint8_t month;
	uint8_t year;
	uint8_t day;
} RTC_date_t;

// --- Time Struct ---
typedef struct{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
} RTC_time_t;


/******************************************
 *              	APIS
 ******************************************/

// --- Init ---
uint8_t ds1307_init(void);

// --- Set/Get Current Time ---
void ds1307_set_current_time(RTC_time_t *pRTCTime);
void ds1307_get_current_time(RTC_time_t *pRTCTime);

// --- Set/Get Current Date ---
void ds1307_set_current_time(RTC_date_t *pRTCDate);
void ds1307_get_current_time(RTC_date_t *pRTCDate);

#endif /* DS1307_H_ */
