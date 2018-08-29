/**
 * @file timer.cpp
 *
 */


#include "oscc_timer.h"

#include "timer.h"
#include "oscc_pid.h"


void start_timer() {
    timer1_init(100, update_pid);
}
