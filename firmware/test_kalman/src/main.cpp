/**
 * @file main.cpp
 *
 */


#include <avr/wdt.h>
#include <oscc_pid.h>

#include "arduino_init.h"
#include "debug.h"
#include "init.h"
#include "timer.h"

int main( void )
{
    init_arduino( );

    init_globals( );

    init_communication_interfaces( );

    start_timer( );
   
    pid_zeroize(&pid, 0);
    pid.proportional_gain = 0.06;
    pid.derivative_gain = 0.008;
    pid.integral_gain = 0.8;

    enabled = true;


    wdt_enable( WDTO_250MS );
    
    DEBUG_PRINTLN( "init complete" );

    while( true )
    {
      wdt_reset();
	
    }
}
