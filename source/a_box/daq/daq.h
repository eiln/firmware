/**
 * @file daq.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Embedded DAQ protocol meant to communicate with a PC dashboard over CAN
 * @version 0.1
 * @date 2021-10-06
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _DAQ_H_
#define _DAQ_H_

#include "can_parse.h"

// Make this match the node name within the daq_config.json
#define NODE_NAME "a_box"

#define DAQ_UPDATE_PERIOD 15 // ms

// BEGIN AUTO VAR COUNT
#define NUM_VARS 12
// END AUTO VAR COUNT

// BEGIN AUTO VAR IDs
#define DAQ_ID_CHARGE_REQUEST_USER 0
#define DAQ_ID_USER_CHARGE_CURRENT_REQUEST 1
#define DAQ_ID_USER_CHARGE_VOLTAGE_REQUEST 2
#define DAQ_ID_TMU_DAQ_OVERRIDE 3
#define DAQ_ID_TMU_DAQ_THERM 4
#define DAQ_ID_TMU_1 5
#define DAQ_ID_TMU_2 6
#define DAQ_ID_TMU_3 7
#define DAQ_ID_TMU_4 8
#define DAQ_ID_BMS_DAQ_OVERRIDE 9
#define DAQ_ID_BMS_DAQ_STAT 10
#define DAQ_ID_CAN_ESR 11
// END AUTO VAR IDs

// BEGIN AUTO FILE STRUCTS
// END AUTO FILE STRUCTS

bool daqInit(q_handle_t* tx_a);
void daqPeriodic();

#endif