/*
 * ms_timer.c
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */

#include "enc_ms_timer.h"
static uint64_t g_millitimer = 0;

uint64_t ms_timer_get_now( void ){
	return g_millitimer;
}

void ms_timer_handler( void ){
	++ g_millitimer;
}

float ms_timer_seconds_elapsed_since( uint64_t time_ms ){
	uint64_t diff = g_millitimer - time_ms;
	return diff * 1e-3;
}

