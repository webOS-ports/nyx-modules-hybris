/*
 *  Copyright (C) 2012 Simon Busch <morphis@gravedo.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef RESUME_HANDLER_H_
#define RESUME_HANDLER_H_

#include <stdbool.h>

void wakeup_system(const char *reason, const char *wakelock_to_release);
bool is_system_suspended(void);

int power_key_resume_handler_init(void);
void power_key_resume_handler_release(void);

#endif

// vim:ts=4:sw=4:noexpandtab
