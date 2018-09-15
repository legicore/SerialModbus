////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusBase.h
 *
 * @author      Martin Legleiter
 *
 * @brief       TODO
 * 
 * @copyright   2018 Martin Legleiter
 * 
 * @license     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_BASE_H__
#define __SERIAL_MODBUS_BASE_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

#include "SerialModbusConfig.h"

/*-----------------------------------------------------------*/

/** TODO */
typedef enum MBFunctionCode_e
{
    /* Bit Data Access */
    READ_COILS                      = 1,
    READ_DISCRETE_INPUTS            = 2,
    WRITE_SINGLE_COIL               = 5,
    WRITE_MULTIPLE_COILS            = 15,

    /* Word Data Access */
    READ_HOLDING_REGISTERS          = 3,
    READ_INPUT_REGISTERS            = 4,
    WRITE_SINGLE_REGISTER           = 6,
    WRITE_MULTIPLE_REGISTERS        = 16,
    MASK_WRITE_REGISTER             = 22,
    READ_WRITE_MULTIPLE_REGISTERS   = 23,
    READ_FIFO_QUEUE                 = 24,

    /* File Record Data Access */
    READ_FILE_RECORD                = 20,
    WRITE_FILE_RECORD               = 21,

    /* Diagnostics */
    READ_EXCEPTION_STATUS           = 7,
    DIAGNOSTIC                      = 8,
    GET_COM_EVENT_COUNTER           = 11,
    GET_COM_EVENT_LOG               = 12,
    REPORT_SLAVE_ID                 = 17
}
MBFunctionCode_t;

/*-----------------------------------------------------------*/

/** TODO */
typedef enum MBSubFunctionCode_e
{
    RETURN_QUERY_DATA                       = 0x00,
    RESTART_COMMUNICATIONS_OPTION           = 0x01,
    RETURN_DIAGNOSTIC_REGISTER              = 0x02,
    CHANGE_ASCII_INPUT_DELIMITER            = 0x03,
    FORCE_LISTEN_ONLY_MODE                  = 0x04,
    CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER  = 0x0A,
    RETURN_BUS_MESSAGE_COUNT                = 0x0B,
    RETURN_BUS_COMMUNICATION_ERROR_COUNT    = 0x0C,
    RETURN_BUS_EXCEPTION_ERROR_COUNT        = 0x0D,
    RETURN_SLAVE_MESSAGE_COUNT              = 0x0E,
    RETURN_SLAVE_NO_RESPONSE_COUNT          = 0x0F,
    RETURN_SLAVE_NAK_COUNT                  = 0x10,
    RETURN_SLAVE_BUSY_COUNT                 = 0x11,
    RETURN_BUS_CHARACTER_OVERRUN_COUNT      = 0x12,
    CLEAR_OVERRUN_COUNTER_AND_FLAG          = 0x14
}
MBSubFunctionCode_t;

/*-----------------------------------------------------------*/

/** TODO */
typedef enum MBException_e
{
    OK = 0x00,

    /* Stansard exception codes */
    ILLEGAL_FUNCTION                        = 0x01,
    ILLEGAL_DATA_ADDRESS                    = 0x02,
    ILLEGAL_DATA_VALUE                      = 0x03,
    SLAVE_DEVICE_FAILURE                    = 0x04,
    ACKNOWLEDGE                             = 0x05,
    SLAVE_DEVICE_BUSY                       = 0x06,
    MEMORY_PARITY_ERROR                     = 0x08,
    GATEWAY_PATH_UNAVAILABLE                = 0x0A,
    GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B,

    /* Non-standard exception codes */
    NOK_BUFFER_OVERFLOW = 0x10,
    NOK_NO_REPLY        = 0x12,
    NOK_BYTE_COUNT      = 0x13,
    NOK_COIL_VALUE      = 0x14,
    NOK_OUTPUT_VALUE    = 0x15,
    NOK_OUTPUT_ADDRESS  = 0x16,
    NOK_QUANTITY        = 0x19,
    NOK_NULL_POINTER    = 0x1A,
    NOK_CHECKSUM        = 0x1B,
    NOK_REQUEST_ENTRY   = 0x1C,
    NOK_PROCESS_STATE   = 0x1D,
    NOK_REGISTER_ACCESS = 0x1E,

    NOK = 0xFF
}
MBException_t;

/** TODO */
typedef MBException_t MBStatus_t;

/*-----------------------------------------------------------*/

/** TODO */
class SerialModbusBase
{
public:

    SerialModbusBase();
    void setSerialCtrl( void (*serialCtrlTx)( void ), void (*serialCtrlRx)( void ) );
#if( configMODE == configMODE_RTU )
    void setInterFrameDelay( uint32_t timeMs );
    void setInterCharacterTimeout( uint32_t timeMs );
#endif
#if( configPROCESS_LOOP_HOOK == 1 )
    void setProcessLoopHook( void (*loopHookFunction)( void ) );
#endif

protected:

    uint8_t pucRequestFrame[ configMAX_FRAME_SIZE ];
    uint8_t pucReplyFrame[ configMAX_FRAME_SIZE ];
    size_t xRequestLength;
    size_t xReplyLength;
    MBException_t xException;
    MBException_t xSetException( MBException_t xExceptionPar );
    MBStatus_t xSetChecksum( uint8_t * pucFrame, size_t * pxFrameLen );
    MBStatus_t xCheckChecksum( uint8_t * pucFrame, size_t xFrameLen );
    void vClearRequestFrame( void );
    void vClearReplyFrame( void );
    bool bReceiveByte( uint8_t * pucReceiveBuffer, size_t * pxBufferLength );
    void vSendData( uint8_t * pucSendBuffer, size_t xBufferLength );
    void (*vSerialCtrlTx)( void );
    void (*vSerialCtrlRx)( void );
    HardwareSerial * pxSerial;
    SoftwareSerial * pxSerialSoftware;
    uint8_t ucRequestByte( size_t xNbr, size_t xOffset = 4 );
    uint16_t usRequestWord( size_t xNbr, size_t xOffset = 4 );
    uint32_t ulRequestDword( size_t xNbr, size_t xOffset = 4 );
    uint64_t uxRequestQword( size_t xNbr, size_t xOffset = 4 );
    uint8_t ucReplyByte( size_t xNbr, size_t xOffset = 3 );
    uint16_t usReplyWord( size_t xNbr, size_t xOffset = 3 );
    uint32_t ulReplyDword( size_t xNbr, size_t xOffset = 3 );
    uint64_t uxReplyQword( size_t xNbr, size_t xOffset = 3 );
#if( configMODE == configMODE_RTU )
    uint16_t usCRC16( uint8_t * pucData, size_t xDataLength );
    uint32_t ulInterFrameDelayUs;
    uint32_t ulInterCharacterTimeoutUs;
    uint32_t ulTimerInterFrameDelayUs;
    uint32_t ulTimerInterCharacterTimeoutUs;
    void vStartInterFrameDelay( void );
    void vStartInterCharacterTimeout( void );
    bool bTimeoutInterFrameDelay( void );
    bool bTimeoutInterCharacterTimeout( void );
#endif
#if( configMODE == configMODE_ASCII )
    uint8_t ucLRC( uint8_t * pucData, size_t xDataLength );
    uint8_t ucByteToAsciiHi( uint8_t ucByte );
    uint8_t ucByteToAsciiLo( uint8_t ucByte );
    uint8_t ucAsciiToByte( uint8_t ucAsciiHi, uint8_t ucAsciiLo );
    MBStatus_t xRtuToAscii( uint8_t * pucRtuFrame, size_t * pxFrameLength );
    MBStatus_t xAsciiToRtu( uint8_t * pucAsciiFrame, size_t * pxFrameLength );
#endif
#if( configPROCESS_LOOP_HOOK == 1 )
    void (*vProcessLoopHook)( void );
#endif
#if( ( configMODE == configMODE_ASCII ) || ( configFC08 == 1 ) )
    char cAsciiInputDelimiter;
#endif
};
/*-----------------------------------------------------------*/

#define ucREQUEST_ID                    pucRequestFrame[ 0 ]
#define ucREQUEST_FUNCTION_CODE         pucRequestFrame[ 1 ]
#define ucREQUEST_BYTE_COUNT            pucRequestFrame[ 2 ]

#define ucREQUEST_ADDRESS_HI            pucRequestFrame[ 2 ]
#define ucREQUEST_ADDRESS_LO            pucRequestFrame[ 3 ]
#define usREQUEST_ADDRESS               ( ( ( uint16_t ) ucREQUEST_ADDRESS_HI << 8 ) | ucREQUEST_ADDRESS_LO )

#define ucREQUEST_QUANTITY_HI           pucRequestFrame[ 4 ]
#define ucREQUEST_QUANTITY_LO           pucRequestFrame[ 5 ]
#define usREQUEST_QUANTITY              ( ( ( uint16_t ) ucREQUEST_QUANTITY_HI << 8 ) | ucREQUEST_QUANTITY_LO )

#define COIL_ON                         0xFF00
#define COIL_OFF                        0x0000
#define ucREQUEST_COIL_VALUE_HI         pucRequestFrame[ 4 ]
#define ucREQUEST_COIL_VALUE_LO         pucRequestFrame[ 5 ]
#define usREQUEST_COIL_VALUE            ( ( ( uint16_t ) ucREQUEST_COIL_VALUE_HI << 8 ) | ucREQUEST_COIL_VALUE_LO )

#define ucREQUEST_REGISTER_VALUE_HI     pucRequestFrame[ 4 ]
#define ucREQUEST_REGISTER_VALUE_LO     pucRequestFrame[ 5 ]
#define usREQUEST_REGISTER_VALUE        ( ( ( uint16_t ) ucREQUEST_REGISTER_VALUE_HI << 8 ) | ucREQUEST_REGISTER_VALUE_LO )

#define ucREQUEST_OUTPUT_VALUE_HI       pucRequestFrame[ 4 ]
#define ucREQUEST_OUTPUT_VALUE_LO       pucRequestFrame[ 5 ]
#define usREQUEST_OUTPUT_VALUE          ( ( ( uint16_t ) ucREQUEST_OUTPUT_VALUE_HI << 8 ) | ucREQUEST_OUTPUT_VALUE_LO )

#define ucREQUEST_BYTE_COUNT_2          pucRequestFrame[ 6 ]

#define ucREQUEST_SUB_FUNCTION_CODE_HI  pucRequestFrame[ 2 ]
#define ucREQUEST_SUB_FUNCTION_CODE_LO  pucRequestFrame[ 3 ]
#define usREQUEST_SUB_FUNCTION_CODE     ( ( ( uint16_t ) ucREQUEST_SUB_FUNCTION_CODE_HI << 8 ) | ucREQUEST_SUB_FUNCTION_CODE_LO )

#define ucREQUEST_DATA_HI               ucReplyFrame[ 4 ]
#define ucREQUEST_DATA_LO               ucReplyFrame[ 5 ]
#define usREQUEST_DATA                  ( ( ( uint16_t ) ucREQUEST_DATA_HI << 8 ) | ucREQUEST_DATA_LO )

#define ucREQUEST_INPUT_DELIMITER_HI    pucRequestFrame[ 4 ]
#define ucREQUEST_INPUT_DELIMITER_LO    pucRequestFrame[ 5 ]
#define usREQUEST_INPUT_DELIMITER       ( ( ( uint16_t ) ucREQUEST_INPUT_DELIMITER_HI << 8 ) | ucREQUEST_INPUT_DELIMITER_LO )

/*-----------------------------------------------------------*/

#define ucREPLY_ID                      pucReplyFrame[ 0 ]
#define ucREPLY_FUNCTION_CODE           pucReplyFrame[ 1 ]
#define ucREPLY_BYTE_COUNT              pucReplyFrame[ 2 ]
#define ucREPLY_ERROR_CODE              pucReplyFrame[ 2 ]

#define ucREPLY_ADDRESS_HI              pucReplyFrame[ 2 ]
#define ucREPLY_ADDRESS_LO              pucReplyFrame[ 3 ]
#define usREPLY_ADDRESS                 ( ( ( uint16_t ) ucREPLY_ADDRESS_HI << 8 ) | ucREPLY_ADDRESS_LO )

#define ucREPLY_QUANTITY_HI             pucReplyFrame[ 4 ]
#define ucREPLY_QUANTITY_LO             pucReplyFrame[ 5 ]
#define usREPLY_QUANTITY                ( ( ( uint16_t ) ucREPLY_QUANTITY_HI << 8 ) | ucREPLY_QUANTITY_LO )

#define ucREPLY_COIL_VALUE_HI           pucReplyFrame[ 4 ]
#define ucREPLY_COIL_VALUE_LO           pucReplyFrame[ 5 ]
#define usREPLY_COIL_VALUE              ( ( ( uint16_t ) ucREPLY_COIL_VALUE_HI << 8 ) | ucREPLY_COIL_VALUE_LO )

#define ucREPLY_REGISTER_VALUE_HI       pucReplyFrame[ 4 ]
#define ucREPLY_REGISTER_VALUE_LO       pucReplyFrame[ 5 ]
#define usREPLY_REGISTER_VALUE          ( ( ( uint16_t ) ucREPLY_REGISTER_VALUE_HI << 8 ) | ucREPLY_REGISTER_VALUE_LO )

#define ucREPLY_OUTPUT_VALUE_HI         pucReplyFrame[ 4 ]
#define ucREPLY_OUTPUT_VALUE_LO         pucReplyFrame[ 5 ]
#define usREPLY_OUTPUT_VALUE            ( ( ( uint16_t ) ucREPLY_OUTPUT_VALUE_HI << 8 ) | ucREPLY_OUTPUT_VALUE_LO )

#define ucREPLY_SUB_FUNCTION_CODE_HI    pucReplyFrame[ 2 ]
#define ucREPLY_SUB_FUNCTION_CODE_LO    pucReplyFrame[ 3 ]
#define usREPLY_SUB_FUNCTION_CODE       ( ( ( uint16_t ) ucREPLY_SUB_FUNCTION_CODE_HI << 8 ) | ucREPLY_SUB_FUNCTION_CODE_LO )

#define ucREPLY_DATA_HI                 pucReplyFrame[ 4 ]
#define ucREPLY_DATA_LO                 pucReplyFrame[ 5 ]
#define usREPLY_DATA                    ( ( ( uint16_t ) ucREPLY_DATA_HI << 8 ) | ucREPLY_DATA_LO )

#define ucREPLY_INPUT_DELIMITER_HI      pucReplyFrame[ 4 ]
#define ucREPLY_INPUT_DELIMITER_LO      pucReplyFrame[ 5 ]
#define usREPLY_INPUT_DELIMITER         ( ( ( uint16_t ) ucREPLY_INPUT_DELIMITER_HI << 8 ) | ucREPLY_INPUT_DELIMITER_LO )

/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_BASE_H__ */
