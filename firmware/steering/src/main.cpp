/**
 * @file main.cpp
 *
 */


#include "arduino_init.h"
#include "communications.h"
#include "debug.h"
#include "init.h"
#include "steering_control.h"
#include "oscc_pid.h"


int main( void )
{
    //Serial.begin(115200);
    //Serial.println("here");
    init_arduino( );

    init_globals( );

    init_devices( );

    init_communication_interfaces( );

    start_timers( );

    pid_zeroize(&pid, 0);
    pid.proportional_gain = 0.06;
    pid.derivative_gain = 0.008;
    pid.integral_gain = 0.8;

    DEBUG_PRINTLN( "init complete" );

    while( true )
    {
        check_for_incoming_message( );

        check_for_faults( );
    }
}
