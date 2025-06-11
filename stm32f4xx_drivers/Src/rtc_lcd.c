/*
 * rtc_lcd.c
 *
 *  Created on: Jun 5, 2025
 *      Author: hugo-juarez
 */

#include <stdio.h>
#include "ds1307.h"

// --- Helper Function Prototypes ---
char* get_day_of_week(uint8_t i);
char* time_to_string(RTC_time_t* pRTCTime);
char* date_to_string(RTC_date_t* pRTCDate);

int main(void){

	RTC_time_t time;
	RTC_date_t date;

	printf("RTC test \n");

	if( ds1307_init() ){
		printf("RTC Init has failed");
		while(1);
	}

	date.day = WEDNESDAY;
	date.date = 15;
	date.month = 6;
	date.year = 25;

	time.hours = 6;
	time.minutes = 20;
	time.seconds = 10;
	time.time_format = TIME_FORMAT_12HRS_AM;

	ds1307_set_current_time(&time);
	ds1307_set_current_date(&date);

	ds1307_get_current_time(&time);
	ds1307_get_current_date(&date);

	char *am_pm;

	if(time.time_format != TIME_FORMAT_24HRS){
		am_pm = (time.time_format == TIME_FORMAT_12HRS_PM) ? "PM" : "AM";
		printf("Current time = %s %s\n", time_to_string(&time), am_pm);
	}else{
		printf("Current time = %s\n", time_to_string(&time));
	}

	printf("Current date = %s <%s>\n", date_to_string(&date), get_day_of_week(date.day));

	while(1);

	return 0;
}

// --- Get day of the week ---
char* get_day_of_week(uint8_t i){
	char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

	return days[i-1];
}

// --- Number to string ---
void number_to_string(uint8_t num, char* buf){
	buf[0] = (num / 10) + '0';
	buf[1] = (num % 10) + '0';
}

// --- Time as hh:mm:ss ---
char* time_to_string(RTC_time_t* pRTCTime){

	static char buf[9];

	buf[2] = buf[5] = ':';

	number_to_string(pRTCTime->hours, buf);
	number_to_string(pRTCTime->minutes, &buf[3]);
	number_to_string(pRTCTime->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;

}

// --- Date as dd/mm/yy ---
char* date_to_string(RTC_date_t* pRTCDate){

	static char buf[9];

	buf[2] = buf[5] = '/';

	number_to_string(pRTCDate->date, buf);
	number_to_string(pRTCDate->month, &buf[3]);
	number_to_string(pRTCDate->year, &buf[6]);

	buf[8] = '\0';

	return buf;

}
