/*
 * rtc_lcd.c
 *
 *  Created on: Jun 5, 2025
 *      Author: hugo-juarez
 */

#include "ds1307.h"
#include <stdio.h>


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

	time.hours = 5;
	time.minutes = 25;
	time.seconds = 41;
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

	return 0;
}


char* get_day_of_week(uint8_t i){
	char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

	return days[i-1];
}
