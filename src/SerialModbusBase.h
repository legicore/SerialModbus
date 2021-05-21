////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusBase.h
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

#ifndef __SERIAL_MODBUS_BASE_H__
#define __SERIAL_MODBUS_BASE_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusConfig.h"

#include <Arduino.h>
#if !defined( ARDUINO_ARCH_RP2040 )
    #include <SoftwareSerial.h>
#endif

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
    RETURN_QUERY_DATA                       = 0,
    RESTART_COMMUNICATIONS_OPTION           = 1,
    RETURN_DIAGNOSTIC_REGISTER              = 2,
    CHANGE_ASCII_INPUT_DELIMITER            = 3,
    FORCE_LISTEN_ONLY_MODE                  = 4,
    CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER  = 10,
    RETURN_BUS_MESSAGE_COUNT                = 11,
    RETURN_BUS_COMMUNICATION_ERROR_COUNT    = 12,
    RETURN_BUS_EXCEPTION_ERROR_COUNT        = 13,
    RETURN_SLAVE_MESSAGE_COUNT              = 14,
    RETURN_SLAVE_NO_RESPONSE_COUNT          = 15,
    RETURN_SLAVE_NAK_COUNT                  = 16,
    RETURN_SLAVE_BUSY_COUNT                 = 17,
    RETURN_BUS_CHARACTER_OVERRUN_COUNT      = 18,
    CLEAR_OVERRUN_COUNTER_AND_FLAG          = 20
}
MBSubFunctionCode_t;

/*-----------------------------------------------------------*/

/** TODO */
typedef enum MBException_e
{
    OK = 0x00,

    /* Standard exception codes. */
    ILLEGAL_FUNCTION                        = 0x01,
    ILLEGAL_DATA_ADDRESS                    = 0x02,
    ILLEGAL_DATA_VALUE                      = 0x03,
    SLAVE_DEVICE_FAILURE                    = 0x04,
    ACKNOWLEDGE                             = 0x05,
    SLAVE_DEVICE_BUSY                       = 0x06,
    MEMORY_PARITY_ERROR                     = 0x08,
    GATEWAY_PATH_UNAVAILABLE                = 0x0A,
    GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B,

    /* Non-standard exception codes. */
    ILLEGAL_REQUEST                         = 0x11,
    CHARACTER_OVERRUN                       = 0x12,
    NO_REPLY                                = 0x13,
    ILLEGAL_CHECKSUM                        = 0x14,
    ILLEGAL_STATE                           = 0x15,
    ILLEGAL_BYTE_COUNT                      = 0x16,
    ILLEGAL_COIL_VALUE                      = 0x17,
    ILLEGAL_OUTPUT_ADDRESS                  = 0x18,
    ILLEGAL_OUTPUT_VALUE                    = 0x19,
    ILLEGAL_QUANTITY                        = 0x1A,
    ILLEGAL_QUERY_DATA                      = 0x1B,
    ILLEGAL_SUB_FUNCTION                    = 0x1C,

#if( configEXTENDED_EXCEPTION_CODES == 1 )

    /* Extended exception codes for slave replies. */
    SLV_ILLEGAL_FUNCTION                    = 0x21,
    SLV_ILLEGAL_STATE                       = 0x22,
    SLV_ILLEGAL_DATA_ADDRESS                = 0x23,
    SLV_ILLEGAL_ACCESS                      = 0x24,
    SLV_ILLEGAL_QUANTITY                    = 0x25,
    SLV_ILLEGAL_COIL_VALUE                  = 0x26,
    SLV_ILLEGAL_ASCII_DELIMITER             = 0x27,
    SLV_ILLEGAL_SUB_FUNCTION                = 0x28,
    SLV_ILLEGAL_DATA_VALUE                  = 0x29,

#endif

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
#if defined( __AVR_ATmega640__  ) || defined( __AVR_ATmega1280__ ) || defined( __AVR_ATmega1281__ ) || defined( __AVR_ATmega2560__ ) || defined( __AVR_ATmega2561__ ) || \
    defined( __AVR_ATmega328P__ ) || defined( __AVR_ATmega168__  ) || defined( __AVR_ATmega8__    ) || \
    defined( __AVR_ATmega32U4__ ) || defined( __AVR_ATmega16U4__ ) || \
    defined( ARDUINO_ARCH_RP2040 )
    bool begin( uint32_t baud, HardwareSerial * serial );
    bool begin( uint32_t baud, HardwareSerial * serial, uint8_t config );
#elif defined( __AVR_ATmega4809__ )
    bool begin( uint32_t baud, UartClass * serial );
    bool begin( uint32_t baud, UartClass * serial, uint32_t config );
#else
    #error the currently selected board is unsuported
#endif
#if !defined( ARDUINO_ARCH_RP2040 )
    bool begin( uint32_t baud, SoftwareSerial * serial );
#endif
    void setSerialCtrl( void (*serialCtrlTx)( void ), void (*serialCtrlRx)( void ) );
#if( configPROCESS_LOOP_HOOK == 1 )
    void setProcessLoopHook( void (*loopHookFunction)( void ) );
#endif
    void setCustomDelay( void (*customDelay)( uint32_t delayUs ) );
#if( configMODE == configMODE_RTU )
    uint32_t getInterCharacterTimeout( void ) const;
    uint32_t getInterFrameDelay( void ) const;
    int8_t setInterCharacterTimeout( uint32_t timeUs );
    int8_t setInterFrameDelay( uint32_t timeUs );
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
#if defined( __AVR_ATmega4809__ )
    UartClass * pxSerial;
#else
    HardwareSerial * pxSerial;
#endif
#if !defined( ARDUINO_ARCH_RP2040 )
    SoftwareSerial * pxSerialSoftware;
#endif
    uint32_t ulSerialConfig;
    uint8_t ucRequestByte( size_t xNbr, size_t xOffset = 4 );
    uint16_t usRequestWord( size_t xNbr, size_t xOffset = 4 );
    uint32_t ulRequestDword( size_t xNbr, size_t xOffset = 4 );
    uint64_t uxRequestQword( size_t xNbr, size_t xOffset = 4 );
    uint8_t ucReplyByte( size_t xNbr, size_t xOffset = 3 );
    uint16_t usReplyWord( size_t xNbr, size_t xOffset = 3 );
    uint32_t ulReplyDword( size_t xNbr, size_t xOffset = 3 );
    uint64_t uxReplyQword( size_t xNbr, size_t xOffset = 3 );
    void vDelayUs( uint32_t ulDelayUs );
    void (*vCustomDelayUs)( uint32_t ulDelayUs );
#if( configMODE == configMODE_RTU )
    uint16_t usCRC16( uint8_t * pucData, size_t xDataLength );
    uint32_t ulInterFrameDelayUs;
    uint32_t ulInterCharacterTimeoutUs;
    uint32_t ulTimerInterFrameDelayUs;
    uint32_t ulTimerInterCharacterTimeoutUs;
    void vStartInterFrameDelay( void );
    void vStartInterCharacterTimeout( void );
    bool bTimeoutInterFrameDelay( void ) const;
    bool bTimeoutInterCharacterTimeout( void ) const;
    bool bCalculateTimeouts( uint32_t ulBaud );
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

#define ucREQUEST_DATA_HI               pucRequestFrame[ 4 ]
#define ucREQUEST_DATA_LO               pucRequestFrame[ 5 ]
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
