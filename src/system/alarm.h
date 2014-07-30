/* @@@LICENSE
*
* Copyright (c) 2014 Simon Busch <morphis@gravedo.de>
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* LICENSE@@@ */

/*
*******************************************
* @file alarm.h
*******************************************
*/

#ifndef _ALARM_H_
#define _ALARM_H_

#include <time.h>

bool android_alarm_open();
void android_alarm_close();
bool android_alarm_set(time_t expiry);
bool android_alarm_clear();
bool android_alarm_read(struct tm *tm_time);
time_t android_alarm_time(time_t *time);

#endif
