/**
 * @file communications.cpp
 *
 */


#include <oscc_can.h>
#include "communications.h"
#include "can_protocols/fault_can_protocol.h"
#include "dtc.h"
#include "globals.h"
#include "mcp_can.h"
#include "oscc_can.h"
#include "vehicles.h"
#include "oscc_pid.h"

#include "debug.h"

static void parse_brake_report(uint8_t *data);

static void parse_steering_report(uint8_t *data);

static void parse_throttle_report(uint8_t *data);


void check_for_module_reports(void) {
    can_frame_s rx_frame;
    can_status_t ret = check_for_rx_frame(g_control_can, &rx_frame);

    if (ret == CAN_RX_FRAME_AVAILABLE) {
        if (rx_frame.id == OSCC_BRAKE_REPORT_CAN_ID) {
            parse_brake_report(rx_frame.data);
        } else if (rx_frame.id == OSCC_STEERING_REPORT_CAN_ID) {
            parse_steering_report(rx_frame.data);
        } else if (rx_frame.id == OSCC_THROTTLE_REPORT_CAN_ID) {
            parse_throttle_report(rx_frame.data);
        } else if (rx_frame.id == OSCC_STEERING_ANGLE_COMMAND_CAN_ID) {
            memcpy(&setpoint, rx_frame.data + 2, 4);
	    can_use_diff = 0;
        } else if (rx_frame.id == OSCC_STEERING_ENABLE_CAN_ID) {
            enabled = 1;
        } else if (rx_frame.id == OSCC_STEERING_DISABLE_CAN_ID || rx_frame.id == OSCC_FAULT_REPORT_CAN_ID) {
            enabled = 0;
        }
    }
}


void republish_obd_frames_to_control_can_bus(void) {
    can_frame_s rx_frame;
    can_status_t ret = check_for_rx_frame(g_obd_can, &rx_frame);

    static unsigned long prev_time, curr_time;
    curr_time = millis();


    if (ret == CAN_RX_FRAME_AVAILABLE) {
        if ((rx_frame.id == KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID) || 
            (rx_frame.id == KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID)) {
            if (rx_frame.id == KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID) {
                curr_angle = rx_frame.data[0] | rx_frame.data[1] << 8;
                curr_angle *= -0.1 * 37 / 520;
	            new_data = 1;
            }
            cli();
            g_control_can.sendMsgBuf(
                    rx_frame.id,
                    CAN_STANDARD,
                    sizeof(rx_frame),
                    (uint8_t * ) & rx_frame.data);
            sei();
            prev_time = curr_time;
        }
    }
}

void publish_torque(float torque) {
    can_frame_s tx_frame;

    memset(&tx_frame, 0, sizeof(tx_frame));
    tx_frame.data[0] = OSCC_MAGIC_BYTE_0;
    tx_frame.data[1] = OSCC_MAGIC_BYTE_1;

    memcpy(tx_frame.data + 2, (unsigned char *) &torque, 4);
    cli();
    g_control_can.sendMsgBuf(
            OSCC_STEERING_COMMAND_CAN_ID,
            CAN_STANDARD,
            sizeof(tx_frame),
            (uint8_t * ) & tx_frame.data);
    sei();
}


static void parse_brake_report(uint8_t *data) {
    oscc_brake_report_s *report = (oscc_brake_report_s *) data;


    if (report->enabled == 1) {
        g_display_state.status_screen.brakes = MODULE_STATUS_ENABLED;
    } else {
        g_display_state.status_screen.brakes = MODULE_STATUS_DISABLED;
    }


    if (report->dtcs != 0) {
        g_display_state.status_screen.brakes = MODULE_STATUS_ERROR;
    }


    if (DTC_CHECK(report->dtcs, OSCC_BRAKE_DTC_INVALID_SENSOR_VAL) != 0) {
        g_display_state.dtc_screen.brakes[OSCC_BRAKE_DTC_INVALID_SENSOR_VAL] = true;
    } else {
        g_display_state.dtc_screen.brakes[OSCC_BRAKE_DTC_INVALID_SENSOR_VAL] = false;
    }

    if (DTC_CHECK(report->dtcs, OSCC_BRAKE_DTC_OPERATOR_OVERRIDE) != 0) {
        g_display_state.dtc_screen.brakes[OSCC_BRAKE_DTC_OPERATOR_OVERRIDE] = true;
    } else {
        g_display_state.dtc_screen.brakes[OSCC_BRAKE_DTC_OPERATOR_OVERRIDE] = false;
    }
}


static void parse_steering_report(uint8_t *data) {
    oscc_steering_report_s *report = (oscc_steering_report_s *) data;


    if (report->enabled == 1) {
        g_display_state.status_screen.steering = MODULE_STATUS_ENABLED;
    } else {
        g_display_state.status_screen.steering = MODULE_STATUS_DISABLED;
    }


    if (report->dtcs != 0) {
        g_display_state.status_screen.steering = MODULE_STATUS_ERROR;
    }


    if (DTC_CHECK(report->dtcs, OSCC_STEERING_DTC_INVALID_SENSOR_VAL) != 0) {
        g_display_state.dtc_screen.steering[OSCC_STEERING_DTC_INVALID_SENSOR_VAL] = true;
    } else {
        g_display_state.dtc_screen.steering[OSCC_STEERING_DTC_INVALID_SENSOR_VAL] = false;
    }

    if (DTC_CHECK(report->dtcs, OSCC_STEERING_DTC_OPERATOR_OVERRIDE) != 0) {
        g_display_state.dtc_screen.steering[OSCC_STEERING_DTC_OPERATOR_OVERRIDE] = true;
    } else {
        g_display_state.dtc_screen.steering[OSCC_STEERING_DTC_OPERATOR_OVERRIDE] = false;
    }
}


static void parse_throttle_report(uint8_t *data) {
    oscc_throttle_report_s *report = (oscc_throttle_report_s *) data;


    if (report->enabled == 1) {
        g_display_state.status_screen.throttle = MODULE_STATUS_ENABLED;
    } else {
        g_display_state.status_screen.throttle = MODULE_STATUS_DISABLED;
    }


    if (report->dtcs != 0) {
        g_display_state.status_screen.throttle = MODULE_STATUS_ERROR;
    }


    if (DTC_CHECK(report->dtcs, OSCC_THROTTLE_DTC_INVALID_SENSOR_VAL) != 0) {
        g_display_state.dtc_screen.throttle[OSCC_THROTTLE_DTC_INVALID_SENSOR_VAL] = true;
    } else {
        g_display_state.dtc_screen.throttle[OSCC_THROTTLE_DTC_INVALID_SENSOR_VAL] = false;
    }

    if (DTC_CHECK(report->dtcs, OSCC_THROTTLE_DTC_OPERATOR_OVERRIDE) != 0) {
        g_display_state.dtc_screen.throttle[OSCC_THROTTLE_DTC_OPERATOR_OVERRIDE] = true;
    } else {
        g_display_state.dtc_screen.throttle[OSCC_THROTTLE_DTC_OPERATOR_OVERRIDE] = false;
    }
}
