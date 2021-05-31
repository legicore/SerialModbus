////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusConfig.h
 * 
 * @author      Martin Legleiter
 * 
 * @brief       TODO
 * 
 * @copyright   (c) 2021 Martin Legleiter
 * 
 * @license     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_CONFIG_H__
#define __SERIAL_MODBUS_CONFIG_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <stddef.h>

#include <Arduino.h>

/*-----------------------------------------------------------*/

#define configMODE_RTU                  1
#define configMODE_ASCII                2
#define configMODE                      configMODE_RTU

#define configMAX_FRAME_SIZE            64

/*-----------------------------------------------------------*/

#define configID_BROADCAST              ( ( uint8_t ) 0 )
#define configID_SLAVE_MAX              ( ( uint8_t ) 247 )

#define configASCII_INPUT_DELIMITER     ( ( char ) '\n' )

#define configTURNAROUND_DELAY_US       ( ( uint32_t ) 200000 )
#define configRESPONSE_TIMEOUT_US       ( ( uint32_t ) 1000000 )

/*-----------------------------------------------------------*/

#define configPROCESS_LOOP_HOOK         1

#define configEXTENDED_EXCEPTION_CODES  0

#define configSLAVE_MULTI_ID            0
#define configMAX_ID_COUNT              8

/*-----------------------------------------------------------*/

#define N 0 /* Not implemented */

/* Bit Data Access */
#define configFC01      N   /* READ_COILS */
#define configFC02      N   /* READ_DISCRETE_INPUTS */
#define configFC05      1   /* WRITE_SINGLE_COIL */
#define configFC15      N   /* WRITE_MULTIPLE_COILS */

/* Word Data Access */
#define configFC03      1   /* READ_HOLDING_REGISTERS */
#define configFC04      1   /* READ_INPUT_REGISTER */
#define configFC06      1   /* WRITE_SINGLE_REGISTER */
#define configFC16      1   /* WRITE_MULTIPLE_REGISTERS */
#define configFC22      N   /* MASK_WRITE_REGISTER */
#define configFC23      N   /* READ_WRITE_MULTIPLE_REGISTERS */
#define configFC24      N   /* READ_FIFO_QUEUE */

/* File Record Data Access */
#define configFC20      N   /* READ_FILE_RECORD */
#define configFC21      N   /* WRITE_FILE_RECORD */

/* Diagnostics */
#define configFC07      N   /* READ_EXCEPTION_STATUS */
#define configFC08      1   /* DIAGNOSTIC */
#define configFC11      N   /* GET_COM_EVENT_COUNTER */
#define configFC12      N   /* GET_COM_EVENT_LOG */
#define configFC17      N   /* REPORT_SLAVE_ID */

/* Diagnostic Sub-Function Codes */
#define configSFC00     1   /* RETURN_QUERY_DATA */
#define configSFC01     1   /* RESTART_COMMUNICATIONS_OPTION */
#define configSFC02     1   /* RETURN_DIAGNOSTIC_REGISTER */
#define configSFC03     1   /* CHANGE_ASCII_INPUT_DELIMITER */
#define configSFC04     1   /* FORCE_LISTEN_ONLY_MODE */
#define configSFC10     1   /* CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER */
#define configSFC11     1   /* RETURN_BUS_MESSAGE_COUNT */
#define configSFC12     1   /* RETURN_BUS_COMMUNICATION_ERROR_COUNT */
#define configSFC13     1   /* RETURN_BUS_EXCEPTION_ERROR_COUNT */
#define configSFC14     1   /* RETURN_SLAVE_MESSAGE_COUNT */
#define configSFC15     1   /* RETURN_SLAVE_NO_RESPONSE_COUNT */
#define configSFC16     1   /* RETURN_SLAVE_NAK_COUNT */
#define configSFC17     1   /* RETURN_SLAVE_BUSY_COUNT */
#define configSFC18     1   /* RETURN_BUS_CHARACTER_OVERRUN_COUNT */
#define configSFC20     1   /* CLEAR_OVERRUN_COUNTER_AND_FLAG */

#undef N /* Not implemented */

/*-----------------------------------------------------------*/

#if( ( configSLAVE_MULTI_ID == 1 ) && ( configFC08 == 1 ) )
    #warning Diagnostics (function code 8) does not work correctly when multi ID support is used!
#endif

#if( ( configFC08 == 1 ) && ( configSFC04 == 1 ) && ( configSFC01 == 0 ) )
    #warning Sub-function code 1 is needed to deavtivate Listen Only Mode (sub-function code 4)!
#endif

/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_CONFIG_H__ */
