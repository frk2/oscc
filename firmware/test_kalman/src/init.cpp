/**
 * @file init.cpp
 *
 */


#include "debug.h"
#include "init.h"
#include "oscc_serial.h"


void init_globals( void )
{

}


void init_communication_interfaces( void )
{
    #ifdef DEBUG
    init_serial();
    #endif


}
