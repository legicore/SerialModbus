////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SerialModbusConfig.h
 * 
 * AUTHOR:      Martin Legleiter
 * 
 * BRIEF:       TODO
 * 
 * COPYRIGHT:   (C) 2025 Martin Legleiter
 * 
 * LICENCE:     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_CONFIG_H__
#define __SERIAL_MODBUS_CONFIG_H__

/*----------------------------------------------------------------------------*/

#include <Arduino.h>

/*----------------------------------------------------------------------------*/

#define configMB_MODE_RTU               1
#define configMB_MODE_ASCII             2
#define configMB_MODE                   configMB_MODE_RTU

#define configMB_FRAME_LEN_MIN          3
#define configMB_FRAME_LEN_MAX          64

/* This value for the Arduino default serial configuration is taken from the 
 * source code (8 data bits, no parity, 1 stop bit). */ 
#define configMB_SERIAL_CONF_DEFAULT    SERIAL_8N1

/*----------------------------------------------------------------------------*/

#define configMB_ID_BROADCAST           0
#define configMB_ID_SERVER_MAX          247

#define configMB_ASCII_INPUT_DELIMITER  '\n'

#define configMB_TURNAROUND_DELAY_MS    200
#define configMB_RESPONSE_TIMEOUT_MS    1000

/*----------------------------------------------------------------------------*/

#define configMB_PROCESS_LOOP_HOOK      1

#define configMB_EXT_EXCEPTION_CODES    0

#define configMB_SERVER_MULTI_ID        0
#define configMB_ID_COUNT_MAX           8

/*----------------------------------------------------------------------------*/

#define N 0 /* Not implemented */

/* Bit Data Access */
#define configMB_FC01       N   /* FC_READ_COILS */
#define configMB_FC02       N   /* FC_READ_DISCRETE_INPUTS */
#define configMB_FC05       1   /* FC_WRITE_SINGLE_COIL */
#define configMB_FC15       N   /* FC_WRITE_MULTIPLE_COILS */

/* Word Data Access */
#define configMB_FC03       1   /* FC_READ_HOLDING_REGISTERS */
#define configMB_FC04       1   /* FC_READ_INPUT_REGISTER */
#define configMB_FC06       1   /* FC_WRITE_SINGLE_REGISTER */
#define configMB_FC16       1   /* FC_WRITE_MULTIPLE_REGISTERS */
#define configMB_FC22       N   /* FC_MASK_WRITE_REGISTER */
#define configMB_FC23       N   /* FC_READ_WRITE_MULTIPLE_REGISTERS */
#define configMB_FC24       N   /* FC_READ_FIFO_QUEUE */

/* File Record Data Access */
#define configMB_FC20       N   /* FC_READ_FILE_RECORD */
#define configMB_FC21       N   /* FC_WRITE_FILE_RECORD */

/* Diagnostics */
#define configMB_FC07       N   /* FC_READ_EXCEPTION_STATUS */
#define configMB_FC08       1   /* FC_DIAGNOSTIC */
#define configMB_FC11       N   /* FC_GET_COM_EVENT_COUNTER */
#define configMB_FC12       N   /* FC_GET_COM_EVENT_LOG */
#define configMB_FC17       N   /* FC_REPORT_SERVER_ID */

/* MB_DIAGNOSTIC Sub-Function Codes */
#define configMB_SFC00      1   /* SFC_RETURN_QUERY_DATA */
#define configMB_SFC01      1   /* SFC_RESTART_COMMUNICATIONS_OPTION */
#define configMB_SFC02      1   /* SFC_RETURN_DIAGNOSTIC_REGISTER */
#define configMB_SFC03      1   /* SFC_CHANGE_ASCII_INPUT_DELIMITER */
#define configMB_SFC04      1   /* SFC_FORCE_LISTEN_ONLY_MODE */
#define configMB_SFC10      1   /* SFC_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER */
#define configMB_SFC11      1   /* SFC_RETURN_BUS_MESSAGE_COUNT */
#define configMB_SFC12      1   /* SFC_RETURN_BUS_COMMUNICATION_ERROR_COUNT */
#define configMB_SFC13      1   /* SFC_RETURN_BUS_EXCEPTION_ERROR_COUNT */
#define configMB_SFC14      1   /* SFC_RETURN_SERVER_MESSAGE_COUNT */
#define configMB_SFC15      1   /* SFC_RETURN_SERVER_NO_RESPONSE_COUNT */
#define configMB_SFC16      1   /* SFC_RETURN_SERVER_NAK_COUNT */
#define configMB_SFC17      1   /* SFC_RETURN_SERVER_BUSY_COUNT */
#define configMB_SFC18      1   /* SFC_RETURN_BUS_CHARACTER_OVERRUN_COUNT */
#define configMB_SFC20      1   /* SFC_CLEAR_OVERRUN_COUNTER_AND_FLAG */

#undef N /* Not implemented */

/*----------------------------------------------------------------------------*/

#if( ( configMB_SERVER_MULTI_ID == 1 ) && ( configMB_FC08 == 1 ) )
    #warning Diagnostics (function code 8) does not work correctly when multi ID support is used!
#endif

#if( ( configMB_FC08 == 1 ) && ( configMB_SFC04 == 1 ) && ( configMB_SFC01 == 0 ) )
    #warning Sub-function code 1 is needed to deactivate Listen Only Mode (sub-function code 4)!
#endif

/*----------------------------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_CONFIG_H__ */
