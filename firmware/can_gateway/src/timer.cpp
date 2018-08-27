/**
 * @file timer.cpp
 *
 */


#include "oscc_timer.h"

#include "timer.h"
#include "oscc_pid.h"


void start_timer() {
    timer1_init(50, update_pid);
}
