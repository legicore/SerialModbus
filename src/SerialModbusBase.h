////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SerialModbusBase.h
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

#ifndef __SERIAL_MODBUS_BASE_H__
#define __SERIAL_MODBUS_BASE_H__

/*----------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusConfig.h"
#include "SerialModbusCompat.h"

#include <Arduino.h>
#if defined( configMB_SERIAL_SW )
    #include <SoftwareSerial.h>
#endif

/*----------------------------------------------------------------------------*/

enum MB_FunctionCode_e
{
    /* Bit Data Access */
    FC_READ_COILS                       = 1,
    FC_READ_DISCRETE_INPUTS             = 2,
    FC_WRITE_SINGLE_COIL                = 5,
    FC_WRITE_MULTIPLE_COILS             = 15,

    /* Word Data Access */
    FC_READ_HOLDING_REGISTERS           = 3,
    FC_READ_INPUT_REGISTERS             = 4,
    FC_WRITE_SINGLE_REGISTER            = 6,
    FC_WRITE_MULTIPLE_REGISTERS         = 16,
    FC_MASK_WRITE_REGISTER              = 22,
    FC_READ_WRITE_MULTIPLE_REGISTERS    = 23,
    FC_READ_FIFO_QUEUE                  = 24,

    /* File Record Data Access */
    FC_READ_FILE_RECORD                 = 20,
    FC_WRITE_FILE_RECORD                = 21,

    /* Diagnostics */
    FC_READ_EXCEPTION_STATUS            = 7,
    FC_DIAGNOSTIC                       = 8,
    FC_GET_COM_EVENT_COUNTER            = 11,
    FC_GET_COM_EVENT_LOG                = 12,
    FC_REPORT_SERVER_ID                 = 17
};

typedef enum MB_FunctionCode_e MB_FunctionCode_t;

/*----------------------------------------------------------------------------*/

enum MB_SubFunctionCode_e
{
    SFC_RETURN_QUERY_DATA                       = 0,
    SFC_RESTART_COMMUNICATIONS_OPTION           = 1,
    SFC_RETURN_DIAGNOSTIC_REGISTER              = 2,
    SFC_CHANGE_ASCII_INPUT_DELIMITER            = 3,
    SFC_FORCE_LISTEN_ONLY_MODE                  = 4,
    SFC_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER  = 10,
    SFC_RETURN_BUS_MESSAGE_COUNT                = 11,
    SFC_RETURN_BUS_COMMUNICATION_ERROR_COUNT    = 12,
    SFC_RETURN_BUS_EXCEPTION_ERROR_COUNT        = 13,
    SFC_RETURN_SERVER_MESSAGE_COUNT             = 14,
    SFC_RETURN_SERVER_NO_RESPONSE_COUNT         = 15,
    SFC_RETURN_SERVER_NAK_COUNT                 = 16,
    SFC_RETURN_SERVER_BUSY_COUNT                = 17,
    SFC_RETURN_BUS_CHARACTER_OVERRUN_COUNT      = 18,
    SFC_CLEAR_OVERRUN_COUNTER_AND_FLAG          = 20
};

typedef enum MB_SubFunctionCode_e MB_SubFunctionCode_t;

/*----------------------------------------------------------------------------*/

enum MB_Status_e
{
    MB_OK = 0x00,

    /* Standard exception codes. */
    MB_ILLEGAL_FUNCTION                         = 0x01,
    MB_ILLEGAL_DATA_ADDRESS                     = 0x02,
    MB_ILLEGAL_DATA_VALUE                       = 0x03,
    MB_SERVER_DEVICE_FAILURE                    = 0x04,
    MB_ACKNOWLEDGE                              = 0x05,
    MB_SERVER_DEVICE_BUSY                       = 0x06,
    MB_MEMORY_PARITY_ERROR                      = 0x08,
    MB_GATEWAY_PATH_UNAVAILABLE                 = 0x0A,
    MB_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND  = 0x0B,

    /* Non-standard exception codes. */
    MB_ILLEGAL_REQUEST                          = 0x11,
    MB_CHARACTER_OVERRUN                        = 0x12,
    MB_NO_REPLY                                 = 0x13,
    MB_ILLEGAL_CHECKSUM                         = 0x14,
    MB_ILLEGAL_STATE                            = 0x15,
    MB_ILLEGAL_BYTE_COUNT                       = 0x16,
    MB_ILLEGAL_COIL_VALUE                       = 0x17,
    MB_ILLEGAL_OUTPUT_ADDRESS                   = 0x18,
    MB_ILLEGAL_OUTPUT_VALUE                     = 0x19,
    MB_ILLEGAL_QUANTITY                         = 0x1A,
    MB_ILLEGAL_QUERY_DATA                       = 0x1B,
    MB_ILLEGAL_SUB_FUNCTION                     = 0x1C,
    MB_ILLEGAL_REPLY_SUB_FUNCTION               = 0x1D,

#if( configMB_EXT_EXCEPTION_CODES == 1 )

    /* Extended exception codes for server replies. */
    MB_SERVER_ILLEGAL_FUNCTION                  = 0x21,
    MB_SERVER_ILLEGAL_STATE                     = 0x22,
    MB_SERVER_ILLEGAL_ACCESS                    = 0x23,
    MB_SERVER_ILLEGAL_QUANTITY                  = 0x24,
    MB_SERVER_ILLEGAL_COIL_VALUE                = 0x25,
    MB_SERVER_ILLEGAL_INPUT_DELIMITER           = 0x26,
    MB_SERVER_ILLEGAL_SUB_FUNCTION              = 0x27,

#endif

    MB_NOK = 0xFE
};

typedef enum MB_Status_e MB_Status_t;
typedef enum MB_Status_e MB_Exception_t;

/*----------------------------------------------------------------------------*/

#define MB_COIL_ON              0xFF00
#define MB_COIL_OFF             0x0000

#define MB_CLEAR_COM_EVENT_LOG  0xFF00

/*----------------------------------------------------------------------------*/

typedef configMB_SERIAL MB_Serial_t;
#if defined( configMB_SERIAL_SW )
    typedef configMB_SERIAL_SW MB_SWSerial_t;
#endif
/*----------------------------------------------------------------------------*/

class SerialModbusBase
{
public:

    SerialModbusBase();
    bool begin( uint32_t baud, MB_Serial_t * serial = &SERIAL_PORT_HARDWARE, uint32_t config = configMB_SERIAL_CONF_DEFAULT );
#if defined( configMB_SERIAL_SW )
    bool begin( uint32_t baud, MB_SWSerial_t * serial );
#endif
    bool setSerialCtrl( void (* serialCtrlTx)( void ), void (* serialCtrlRx)( void ) );
#if( configMB_PROCESS_LOOP_HOOK == 1 )
    void setProcessLoopHook( void (* loopHookFunction)( void ) );
#endif
    void setCustomDelay( void (* customDelay)( uint32_t delayUs ) );
    const char * getExceptionString( MB_Exception_t exception );
    uint32_t getInterCharacterTimeout( void );
    uint32_t getInterFrameDelay( void );
    bool setInterCharacterTimeout( uint32_t timeUs );
    bool setInterFrameDelay( uint32_t timeUs );

protected:

    uint8_t pucRequestFrame[ configMB_FRAME_LEN_MAX ];
    uint8_t pucReplyFrame[ configMB_FRAME_LEN_MAX ];
    size_t xRequestLength;
    size_t xReplyLength;
    MB_Status_t xStatus;
    MB_Status_t xSetException( MB_Exception_t xExceptionPar );
    MB_Status_t xSetChecksum( uint8_t * pucFrame, size_t * pxFrameLen );
    MB_Status_t xCheckChecksum( uint8_t * pucFrame, size_t xFrameLen );
    void vClearRequestFrame( void );
    void vClearReplyFrame( void );
    bool bReceiveByte( uint8_t * pucReceiveBuffer, size_t * pxBufferLength );
    size_t xSendData( uint8_t * pucSendBuffer, size_t xBufferLength );
    void (* vSerialCtrlTx)( void );
    void (* vSerialCtrlRx)( void );
    MB_Serial_t * pxSerial;
#if defined( configMB_SERIAL_SW )
    MB_SWSerial_t * pxSWSerial;
#endif
    uint8_t ucRequestByte( size_t xNbr, size_t xOffset = 4 );
    uint16_t usRequestWord( size_t xNbr, size_t xOffset = 4 );
    uint32_t ulRequestDword( size_t xNbr, size_t xOffset = 4 );
    uint64_t uxRequestQword( size_t xNbr, size_t xOffset = 4 );
    uint8_t ucReplyByte( size_t xNbr, size_t xOffset = 3 );
    uint16_t usReplyWord( size_t xNbr, size_t xOffset = 3 );
    uint32_t ulReplyDword( size_t xNbr, size_t xOffset = 3 );
    uint64_t uxReplyQword( size_t xNbr, size_t xOffset = 3 );
    void vDelayUs( uint32_t ulDelayUs );
    void (* vCustomDelayUs)( uint32_t ulDelayUs );
    uint16_t usCRC16( uint8_t * pucData, size_t xDataLength );
    uint32_t ulInterFrameDelayUs;
    uint32_t ulInterCharacterTimeoutUs;
    uint32_t ulTimerInterFrameDelayUs;
    uint32_t ulTimerInterCharacterTimeoutUs;
    void vStartInterFrameDelay( void );
    void vStartInterCharacterTimeout( void );
    bool bTimeoutInterFrameDelay( void );
    bool bTimeoutInterCharacterTimeout( void );
    bool bCalculateTimeouts( uint32_t ulBaud, uint32_t ulConfig );
    uint8_t ucLRC( uint8_t * pucData, size_t xDataLength );
    uint8_t ucByteToAsciiHi( uint8_t ucByte );
    uint8_t ucByteToAsciiLo( uint8_t ucByte );
    uint8_t ucAsciiToByte( uint8_t ucAsciiHi, uint8_t ucAsciiLo );
    MB_Status_t xRtuToAscii( uint8_t * pucRtuFrame, size_t * pxFrameLength );
    MB_Status_t xAsciiToRtu( uint8_t * pucAsciiFrame, size_t * pxFrameLength );
#if( configMB_PROCESS_LOOP_HOOK == 1 )
    void (* vProcessLoopHook)( void );
#endif
    char cAsciiInputDelimiter;
    size_t xChecksumLength;
};
/*----------------------------------------------------------------------------*/

#define ucREQUEST_ID                    pucRequestFrame[ 0 ]
#define ucREQUEST_FUNCTION_CODE         pucRequestFrame[ 1 ]
#define ucREQUEST_BYTE_COUNT            pucRequestFrame[ 2 ]

#define ucREQUEST_ADDRESS_HI            pucRequestFrame[ 2 ]
#define ucREQUEST_ADDRESS_LO            pucRequestFrame[ 3 ]
#define usREQUEST_ADDRESS               ( ( ( uint16_t ) pucRequestFrame[ 2 ] << 8 ) | pucRequestFrame[ 3 ] )

#define ucREQUEST_QUANTITY_HI           pucRequestFrame[ 4 ]
#define ucREQUEST_QUANTITY_LO           pucRequestFrame[ 5 ]
#define usREQUEST_QUANTITY              ( ( ( uint16_t ) pucRequestFrame[ 4 ] << 8 ) | pucRequestFrame[ 5 ] )

#define ucREQUEST_COIL_VALUE_HI         pucRequestFrame[ 4 ]
#define ucREQUEST_COIL_VALUE_LO         pucRequestFrame[ 5 ]
#define usREQUEST_COIL_VALUE            ( ( ( uint16_t ) pucRequestFrame[ 4 ] << 8 ) | pucRequestFrame[ 5 ] )

#define ucREQUEST_REGISTER_VALUE_HI     pucRequestFrame[ 4 ]
#define ucREQUEST_REGISTER_VALUE_LO     pucRequestFrame[ 5 ]
#define usREQUEST_REGISTER_VALUE        ( ( ( uint16_t ) pucRequestFrame[ 4 ] << 8 ) | pucRequestFrame[ 5 ] )

#define ucREQUEST_OUTPUT_VALUE_HI       pucRequestFrame[ 4 ]
#define ucREQUEST_OUTPUT_VALUE_LO       pucRequestFrame[ 5 ]
#define usREQUEST_OUTPUT_VALUE          ( ( ( uint16_t ) pucRequestFrame[ 4 ] << 8 ) | pucRequestFrame[ 5 ] )

#define ucREQUEST_BYTE_COUNT_FC16       pucRequestFrame[ 6 ]

#define ucREQUEST_SUB_FUNCTION_CODE_HI  pucRequestFrame[ 2 ]
#define ucREQUEST_SUB_FUNCTION_CODE_LO  pucRequestFrame[ 3 ]
#define usREQUEST_SUB_FUNCTION_CODE     ( ( ( uint16_t ) pucRequestFrame[ 2 ] << 8 ) | pucRequestFrame[ 3 ] )

#define ucREQUEST_DATA_HI               pucRequestFrame[ 4 ]
#define ucREQUEST_DATA_LO               pucRequestFrame[ 5 ]
#define usREQUEST_DATA                  ( ( ( uint16_t ) pucRequestFrame[ 4 ] << 8 ) | pucRequestFrame[ 5 ] )

#define ucREQUEST_INPUT_DELIMITER_HI    pucRequestFrame[ 4 ]
#define ucREQUEST_INPUT_DELIMITER_LO    pucRequestFrame[ 5 ]
#define usREQUEST_INPUT_DELIMITER       ( ( ( uint16_t ) pucRequestFrame[ 4 ] << 8 ) | pucRequestFrame[ 5 ] )

#define ucREQUEST_ADDRESS_WRITE_HI      pucRequestFrame[ 6 ]
#define ucREQUEST_ADDRESS_WRITE_LO      pucRequestFrame[ 7 ]
#define usREQUEST_ADDRESS_WRITE         ( ( ( uint16_t ) pucRequestFrame[ 6 ] << 8 ) | pucRequestFrame[ 7 ] )

#define ucREQUEST_QUANTITY_WRITE_HI     pucRequestFrame[ 8 ]
#define ucREQUEST_QUANTITY_WRITE_LO     pucRequestFrame[ 9 ]
#define usREQUEST_QUANTITY_WRITE        ( ( ( uint16_t ) pucRequestFrame[ 8 ] << 8 ) | pucRequestFrame[ 9 ] )

#define ucREQUEST_BYTE_COUNT_FC23       pucRequestFrame[ 10 ]

/*----------------------------------------------------------------------------*/

#define ucREPLY_ID                      pucReplyFrame[ 0 ]
#define ucREPLY_FUNCTION_CODE           pucReplyFrame[ 1 ]
#define ucREPLY_BYTE_COUNT              pucReplyFrame[ 2 ]
#define ucREPLY_ERROR_CODE              pucReplyFrame[ 2 ]

#define ucREPLY_ADDRESS_HI              pucReplyFrame[ 2 ]
#define ucREPLY_ADDRESS_LO              pucReplyFrame[ 3 ]
#define usREPLY_ADDRESS                 ( ( ( uint16_t ) pucReplyFrame[ 2 ] << 8 ) | pucReplyFrame[ 3 ] )

#define ucREPLY_QUANTITY_HI             pucReplyFrame[ 4 ]
#define ucREPLY_QUANTITY_LO             pucReplyFrame[ 5 ]
#define usREPLY_QUANTITY                ( ( ( uint16_t ) pucReplyFrame[ 4 ] << 8 ) | pucReplyFrame[ 5 ] )

#define ucREPLY_COIL_VALUE_HI           pucReplyFrame[ 4 ]
#define ucREPLY_COIL_VALUE_LO           pucReplyFrame[ 5 ]
#define usREPLY_COIL_VALUE              ( ( ( uint16_t ) pucReplyFrame[ 4 ] << 8 ) | pucReplyFrame[ 5 ] )

#define ucREPLY_REGISTER_VALUE_HI       pucReplyFrame[ 4 ]
#define ucREPLY_REGISTER_VALUE_LO       pucReplyFrame[ 5 ]
#define usREPLY_REGISTER_VALUE          ( ( ( uint16_t ) pucReplyFrame[ 4 ] << 8 ) | pucReplyFrame[ 5 ] )

#define ucREPLY_OUTPUT_VALUE_HI         pucReplyFrame[ 4 ]
#define ucREPLY_OUTPUT_VALUE_LO         pucReplyFrame[ 5 ]
#define usREPLY_OUTPUT_VALUE            ( ( ( uint16_t ) pucReplyFrame[ 4 ] << 8 ) | pucReplyFrame[ 5 ] )

#define ucREPLY_SUB_FUNCTION_CODE_HI    pucReplyFrame[ 2 ]
#define ucREPLY_SUB_FUNCTION_CODE_LO    pucReplyFrame[ 3 ]
#define usREPLY_SUB_FUNCTION_CODE       ( ( ( uint16_t ) pucReplyFrame[ 2 ] << 8 ) | pucReplyFrame[ 3 ] )

#define ucREPLY_DATA_HI                 pucReplyFrame[ 4 ]
#define ucREPLY_DATA_LO                 pucReplyFrame[ 5 ]
#define usREPLY_DATA                    ( ( ( uint16_t ) pucReplyFrame[ 4 ] << 8 ) | pucReplyFrame[ 5 ] )

#define ucREPLY_INPUT_DELIMITER_HI      pucReplyFrame[ 4 ]
#define ucREPLY_INPUT_DELIMITER_LO      pucReplyFrame[ 5 ]
#define usREPLY_INPUT_DELIMITER         ( ( ( uint16_t ) pucReplyFrame[ 4 ] << 8 ) | pucReplyFrame[ 5 ] )

/*----------------------------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_BASE_H__ */
