////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusConfig.h
 * 
 * @author      Martin Legleiter
 * 
 * @brief       TODO
 * 
 * @copyright   (c) 2018 Martin Legleiter
 * 
 * @license     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_CONFIG_H__
#define __SERIAL_MODBUS_CONFIG_H__

/*-----------------------------------------------------------*/

#define configMODE_RTU                  1
#define configMODE_ASCII                2
#define configMODE                      configMODE_RTU

#define configMAX_FRAME_SIZE            ( 64 )

#define configID_BROADCAST              ( ( uint8_t ) 0 )
#define configID_SLAVE_MAX              ( ( uint8_t ) 247 )

#define configASCII_INPUT_DELIMITER     ( '\n' )

/*-----------------------------------------------------------*/

#define configINTER_CHARACTER_TIMEOUT_US    750
#define configINTER_FRAME_DELAY_US          1750
#define configTURNAROUND_DELAY_US           200000
#define configRESPONSE_TIMEOUT_US           1000000

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
#define configFC08      0   /* DIAGNOSTIC */
#define configFC11      N   /* GET_COM_EVENT_COUNTER */
#define configFC12      N   /* GET_COM_EVENT_LOG */
#define configFC17      N   /* REPORT_SLAVE_ID */

#undef N /* Not implemented */

/*-----------------------------------------------------------*/

#define configPROCESS_LOOP_HOOK 1

/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_CONFIG_H__ */
