/**
 * @file globals.h
 * @brief Module globals.
 *
 */


#ifndef _OSCC_KIA_SOUL_CAN_GATEWAY_GLOBALS_H_
#define _OSCC_KIA_SOUL_CAN_GATEWAY_GLOBALS_H_


#include "mcp_can.h"
#include "ssd1325.h"

#include "display.h"
#include "oscc_timer.h"
#include "oscc_can.h"
#include "oscc_serial.h"
#include "vehicles.h"
#include "oscc_pid.h"
/*
 * @brief Chip select pin of the OBD CAN IC.
 *
 */
#define PIN_OBD_CAN_CHIP_SELECT ( 9 )

/*
 * @brief Chip select pin of the Control CAN IC.
 *
 */
#define PIN_CONTROL_CAN_CHIP_SELECT ( 10 )

/*
 * @brief SPI CS pin to display.
 *
 */
#define PIN_DISPLAY_CHIP_SELECT ( 8 )


#ifdef GLOBAL_DEFINED
    MCP_CAN g_obd_can( PIN_OBD_CAN_CHIP_SELECT );
    MCP_CAN g_control_can( PIN_CONTROL_CAN_CHIP_SELECT );
    #ifndef TESTS
       SSD1325 g_display( PIN_DISPLAY_CHIP_SELECT );
    #endif

    #define EXTERN
#else
    extern MCP_CAN g_obd_can;
    extern MCP_CAN g_control_can;
    extern SSD1325 g_display;

    #define EXTERN extern
#endif


EXTERN kia_soul_gateway_display_state_s g_display_state;
EXTERN pid_s g_long_pid;
EXTERN oscc_long_speed_traj_s g_speed_trajectory;
EXTERN unsigned long g_last_trajectory_update_time;
EXTERN double g_curr_speed;
EXTERN bool g_new_data;
EXTERN unsigned long g_last_long_update_time;



#endif /* _OSCC_KIA_SOUL_CAN_GATEWAY_GLOBALS_H_ */
