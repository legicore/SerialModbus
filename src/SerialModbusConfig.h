///////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusConfig.h
 *
 * @author      legicore
 *
 * @brief       xxx
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_CONFIG_H__
#define __SERIAL_MODBUS_CONFIG_H__

/*-----------------------------------------------------------*/

#define configMODE_RTU                  1
#define configMODE_ASCII                2
#define configMODE                      configMODE_RTU

#define configMAX_FRAME_SIZE            64

#define configID_BROADCAST              0
#define configID_SLAVE_MAX              247

#define configASCII_INPUT_DELIMITER		'\n'

/*-----------------------------------------------------------*/

#define configINTER_CHARACTER_TIMEOUT_US	1500    /* Orig. 750 */
#define configINTER_FRAME_DELAY_US          3500    /* Orig. 1750 */
#define configTURNAROUND_DELAY_US           200000
#define configRESPONSE_TIMEOUT_US           1000000

/*-----------------------------------------------------------*/

#define N 0 /* Not implemented */

/* BIT DATA ACCESS */
#define configFC01      N	/* READ_COILS */
#define configFC02      N   /* READ_DISCRETE_INPUTS */
#define configFC05      1   /* WRITE_SINGLE_COIL */
#define configFC15      N   /* WRITE_MULTIPLE_COILS */

/* WORD DATA ACCESS */
#define configFC03      1   /* READ_HOLDING_REGISTERS */
#define configFC04      1   /* READ_INPUT_REGISTER */
#define configFC06      1   /* WRITE_SINGLE_REGISTER */
#define configFC16      1   /* WRITE_MULTIPLE_REGISTERS */
#define configFC22      N   /* MASK_WRITE_REGISTER */
#define configFC23      N   /* READ_WRITE_MULTIPLE_REGISTERS */
#define configFC24      N   /* READ_FIFO_QUEUE */

/* FILE RECORD DATA ACCESS */
#define configFC20      N   /* READ_FILE_RECORD */
#define configFC21      N   /* WRITE_FILE_RECORD */

/* DIAGNOSTICS */
#define configFC07      N   /* READ_EXCEPTION_STATUS */
#define configFC08      0   /* DIAGNOSTIC */
#define configFC11      N   /* GET_COM_EVENT_COUNTER */
#define configFC12      N   /* GET_COM_EVENT_LOG */
#define configFC17      N   /* REPORT_SLAVE_ID */

#undef N /* Not implemented */

/*-----------------------------------------------------------*/

#define configPROCESS_LOOP_HOOK		1

/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_CONFIG_H__ */
