////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SerialModbusBase.cpp
 * 
 * AUTHOR:      Martin Legleiter
 * 
 * BRIEF:       TODO
 * 
 * COPYRIGHT:   (C) 2025 Martin Legleiter
 * 
 * LICENCE:     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusConfig.h"
#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"

#include <Arduino.h>
#if defined( configMB_SERIAL_SW )
    #include <SoftwareSerial.h>
#endif

/*----------------------------------------------------------------------------*/

struct MB_ExceptionString_s
{
    MB_Exception_t xException;
    const char * pcString;
};

typedef struct MB_ExceptionString_s MB_ExceptionString_t;

/*----------------------------------------------------------------------------*/

static const MB_ExceptionString_t pxExceptionStrings[] = {

    { MB_OK, "OK" },

    /* Standard exception codes. */
    { MB_ILLEGAL_FUNCTION,                          "ILLEGAL_FUNCTION" },
    { MB_ILLEGAL_DATA_ADDRESS,                      "ILLEGAL_DATA_ADDRESS" },
    { MB_ILLEGAL_DATA_VALUE,                        "ILLEGAL_DATA_VALUE" },
    { MB_SERVER_DEVICE_FAILURE,                     "SERVER_DEVICE_FAILURE" },
    { MB_ACKNOWLEDGE,                               "ACKNOWLEDGE" },
    { MB_SERVER_DEVICE_BUSY,                        "SERVER_DEVICE_BUSY" },
    { MB_MEMORY_PARITY_ERROR,                       "MEMORY_PARITY_ERROR" },
    { MB_GATEWAY_PATH_UNAVAILABLE,                  "GATEWAY_PATH_UNAVAILABLE" },
    { MB_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND,   "GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND" },

    /* Non-standard exception codes. */
    { MB_ILLEGAL_REQUEST,                           "ILLEGAL_REQUEST" },
    { MB_CHARACTER_OVERRUN,                         "CHARACTER_OVERRUN" },
    { MB_NO_REPLY,                                  "NO_REPLY" },
    { MB_ILLEGAL_CHECKSUM,                          "ILLEGAL_CHECKSUM" },
    { MB_ILLEGAL_STATE,                             "ILLEGAL_STATE" },
    { MB_ILLEGAL_BYTE_COUNT,                        "ILLEGAL_BYTE_COUNT" },
    { MB_ILLEGAL_COIL_VALUE,                        "ILLEGAL_COIL_VALUE" },
    { MB_ILLEGAL_OUTPUT_ADDRESS,                    "ILLEGAL_OUTPUT_ADDRESS" },
    { MB_ILLEGAL_OUTPUT_VALUE,                      "ILLEGAL_OUTPUT_VALUE" },
    { MB_ILLEGAL_QUANTITY,                          "ILLEGAL_QUANTITY" },
    { MB_ILLEGAL_QUERY_DATA,                        "ILLEGAL_QUERY_DATA" },
    { MB_ILLEGAL_SUB_FUNCTION,                      "ILLEGAL_SUB_FUNCTION" },
    { MB_ILLEGAL_REPLY_SUB_FUNCTION,                "ILLEGAL_REPLY_SUB_FUNCTION" },

#if( configMB_EXT_EXCEPTION_CODES == 1 )

    /* Extended exception codes for server replies. */
    { MB_SERVER_ILLEGAL_FUNCTION,                   "SERVER_ILLEGAL_FUNCTION" },
    { MB_SERVER_ILLEGAL_STATE,                      "SERVER_ILLEGAL_STATE" },
    { MB_SERVER_ILLEGAL_ACCESS,                     "SERVER_ILLEGAL_ACCESS" },
    { MB_SERVER_ILLEGAL_QUANTITY,                   "SERVER_ILLEGAL_QUANTITY" },
    { MB_SERVER_ILLEGAL_COIL_VALUE,                 "SERVER_ILLEGAL_COIL_VALUE" },
    { MB_SERVER_ILLEGAL_INPUT_DELIMITER,            "SERVER_ILLEGAL_INPUT_DELIMITER" },
    { MB_SERVER_ILLEGAL_SUB_FUNCTION,               "SERVER_ILLEGAL_SUB_FUNCTION" },

#endif

    { MB_NOK, "NOK" },

    /* Marks the end of the list. */
    { ( MB_Exception_t ) 0xFF, NULL }
};
/*----------------------------------------------------------------------------*/

SerialModbusBase::SerialModbusBase()
{
    xRequestLength = 0;
    xReplyLength = 0;

    pxSerial = NULL;
    #if defined( configMB_SERIAL_SW )
    {
        pxSWSerial = NULL;
    }
    #endif

    vSerialCtrlTx = NULL;
    vSerialCtrlRx = NULL;

    xStatus = MB_OK;

    ulInterFrameDelayUs = 0;
    ulTimerInterFrameDelayUs = 0;
    ulInterCharacterTimeoutUs = 0;
    ulTimerInterCharacterTimeoutUs = 0;

    #if( configMB_PROCESS_LOOP_HOOK == 1 )
    {
        vProcessLoopHook = NULL;
    }
    #endif

    cAsciiInputDelimiter = configMB_ASCII_INPUT_DELIMITER;

    vCustomDelayUs = NULL;

    xChecksumLength = 2;
    #if( configMB_MODE == configMB_MODE_ASCII )
    {
        xChecksumLength = 1;
    }
    #endif
}
/*----------------------------------------------------------------------------*/

bool SerialModbusBase::begin( uint32_t baud, MB_Serial_t * serial, uint32_t config )
{
    if( ( baud == 0 ) || ( serial == NULL ) )
    {
        return false;
    }

    pxSerial = serial;
    pxSerial->begin( baud, config );

    #if( configMB_MODE == configMB_MODE_RTU )
    {
        if( bCalculateTimeouts( baud, config ) != true )
        {
            return false;
        }
    }
    #endif

    return true;
}
/*----------------------------------------------------------------------------*/

#if defined( configMB_SERIAL_SW )

    bool SerialModbusBase::begin( uint32_t baud, MB_SWSerial_t * serial )
    {
        if( ( baud == 0 ) || ( serial == NULL ) )
        {
            return false;
        }

        pxSWSerial = serial;
        pxSWSerial->begin( baud );

        #if( configMB_MODE == configMB_MODE_RTU )
        {
            if( bCalculateTimeouts( baud, configMB_SERIAL_CONF_DEFAULT ) != true )
            {
                return false;
            }
        }
        #endif

        return true;
    }

#endif
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusBase::xSetChecksum( uint8_t * pucFrame, size_t * pxFrameLength )
{
    uint16_t usTempChecksum = 0;

    if( ( pucFrame != NULL ) && ( *pxFrameLength >= configMB_FRAME_LEN_MIN ) )
    {
        #if( configMB_MODE == configMB_MODE_RTU )
        {
            usTempChecksum = usCRC16( pucFrame, *pxFrameLength );
            pucFrame[ (*pxFrameLength)++ ] =  lowByte( usTempChecksum );
            pucFrame[ (*pxFrameLength)++ ] = highByte( usTempChecksum );

            return MB_OK;
        }
        #endif

        #if( configMB_MODE == configMB_MODE_ASCII )
        {
            usTempChecksum = ( uint16_t ) ucLRC( pucFrame, *pxFrameLength );
            pucFrame[ (*pxFrameLength)++ ] = ( uint8_t ) usTempChecksum;

            return MB_OK;
        }
        #endif
    }

    return MB_NOK;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusBase::xCheckChecksum( uint8_t * pucFrame, size_t xFrameLength )
{
    uint16_t usTempChecksum = 0;

    if( ( pucFrame != NULL ) && ( xFrameLength >= configMB_FRAME_LEN_MIN ) )
    {
        #if( configMB_MODE == configMB_MODE_RTU )
        {
            usTempChecksum = usCRC16( pucFrame, xFrameLength - 2 );
            if( pucFrame[ xFrameLength - 2 ] == lowByte( usTempChecksum ) )
            {
                if( pucFrame[ xFrameLength - 1 ] == highByte( usTempChecksum ) )
                {
                    return MB_OK;
                }
            }
        }
        #endif

        #if( configMB_MODE == configMB_MODE_ASCII )
        {
            usTempChecksum = ( uint16_t ) ucLRC( pucFrame, xFrameLength - 1 );
            if( pucFrame[ xFrameLength - 1 ] == ( uint8_t ) usTempChecksum )
            {
                return MB_OK;
            }
        }
        #endif
    }

    return MB_NOK;
}
/*----------------------------------------------------------------------------*/

uint16_t SerialModbusBase::usCRC16( uint8_t * pucData, size_t xDataLength )
{
    uint16_t usCrc = 0xFFFF;

    for ( size_t xIndex = 0; xIndex < xDataLength; xIndex++ )
    {
        usCrc ^= ( uint16_t ) pucData[ xIndex ];

        for( size_t xBit = 8; xBit != 0; xBit-- )
        {
            if( ( usCrc & 0x0001 ) != 0 )
            {
                usCrc >>= 1;
                usCrc ^= 0xA001;
            }
            else
            {
                usCrc >>= 1;
            }
        }
    }

    return usCrc;
}
/*----------------------------------------------------------------------------*/

uint8_t SerialModbusBase::ucLRC( uint8_t * pucData, size_t xDataLength )
{
    uint8_t ucLrc = 0;

    while( xDataLength-- > 0 )
    {
        ucLrc += *pucData++;
    }

    return ( uint8_t ) ( -( ( int8_t ) ucLrc ) );
}
/*----------------------------------------------------------------------------*/

uint8_t SerialModbusBase::ucByteToAsciiHi( uint8_t ucByte )
{
    return ( ucByte >> 4 ) + ( ( ( ucByte >> 4 ) < 10 ) ? 48 : 55 );
}
/*----------------------------------------------------------------------------*/

uint8_t SerialModbusBase::ucByteToAsciiLo( uint8_t ucByte )
{
    return ( ucByte & 0x0F ) + ( ( ( ucByte & 0x0F ) < 10 ) ? 48 : 55 );
}
/*----------------------------------------------------------------------------*/

uint8_t SerialModbusBase::ucAsciiToByte( uint8_t ucAsciiHi, uint8_t ucAsciiLo )
{
    return ( ( ucAsciiHi - ( ( ucAsciiHi < 64 ) ? 48 : 55 ) ) << 4 ) | ( ucAsciiLo - ( ( ucAsciiLo < 64 ) ? 48 : 55 ) );
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusBase::xRtuToAscii( uint8_t * pucFrame, size_t * pxFrameLength )
{
    if( ( pucFrame == NULL ) || ( *pxFrameLength < configMB_FRAME_LEN_MIN ) )
    {
        return MB_NOK;
    }

    pucFrame[ ( *pxFrameLength * 2 ) + 2 ] = ( uint8_t ) cAsciiInputDelimiter;
    pucFrame[ ( *pxFrameLength * 2 ) + 1 ] = ( uint8_t ) '\r';

    for( size_t i = *pxFrameLength - 1; ( i + 1 ) > 0; i-- )
    {
        pucFrame[ ( i * 2 ) + 2 ] = ucByteToAsciiLo( pucFrame[ i ] );
        pucFrame[ ( i * 2 ) + 1 ] = ucByteToAsciiHi( pucFrame[ i ] );
    }

    pucFrame[ 0 ] = ( uint8_t ) ':';

    *pxFrameLength = ( *pxFrameLength * 2 ) + 3;

    return MB_OK;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusBase::xAsciiToRtu( uint8_t * pucFrame, size_t * pxFrameLength )
{
    if( ( pucFrame == NULL ) || ( *pxFrameLength < ( configMB_FRAME_LEN_MIN * 2 ) ) )
    {
        return MB_NOK;
    }

    *pxFrameLength = ( *pxFrameLength - 3 ) / 2;

    for( size_t i = 0; i < *pxFrameLength; i++ )
    {
        pucFrame[ i ] = ucAsciiToByte( pucFrame[ ( i * 2 ) + 1 ], pucFrame[ ( i * 2 ) + 2 ] );
    }

    return MB_OK;
}
/*----------------------------------------------------------------------------*/

void SerialModbusBase::vClearReplyFrame( void )
{
    // ( void ) memset( pucReplyFrame, 0x00, xReplyLength );
    xReplyLength = 0;
}
/*----------------------------------------------------------------------------*/

void SerialModbusBase::vClearRequestFrame( void )
{
    // ( void ) memset( pucRequestFrame, 0x00, xRequestLength );
    xRequestLength = 0;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusBase::setSerialCtrl( void (* serialCtrlTx)( void ), void (* serialCtrlRx)( void ) )
{
    if( ( serialCtrlTx != NULL ) && ( serialCtrlRx != NULL ) )
    {
        vSerialCtrlTx = serialCtrlTx;
        vSerialCtrlRx = serialCtrlRx;

        /* Set Rx mode to have an defined initialization status. */
        ( vSerialCtrlRx )();

        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

uint8_t SerialModbusBase::ucRequestByte( size_t xNbr, size_t xOffset )
{
    return pucRequestFrame[ xNbr + xOffset ];
}
/*----------------------------------------------------------------------------*/

uint16_t SerialModbusBase::usRequestWord( size_t xNbr, size_t xOffset )
{
    return ( ( uint16_t ) ucRequestByte( xNbr, xOffset + xNbr ) << 8 ) | ucRequestByte( xNbr + 1, xOffset + xNbr );
}
/*----------------------------------------------------------------------------*/

uint32_t SerialModbusBase::ulRequestDword( size_t xNbr, size_t xOffset )
{
    return ( ( uint32_t ) usRequestWord( xNbr, xOffset ) << 16 ) | usRequestWord( xNbr + 2, xOffset );
}
/*----------------------------------------------------------------------------*/

uint64_t SerialModbusBase::uxRequestQword( size_t xNbr, size_t xOffset )
{
    return ( ( uint64_t ) ulRequestDword( xNbr, xOffset ) << 32 ) | ulRequestDword( xNbr + 4, xOffset );
}
/*----------------------------------------------------------------------------*/

uint8_t SerialModbusBase::ucReplyByte( size_t xNbr, size_t xOffset )
{
    return pucReplyFrame[ xNbr + xOffset ];
}
/*----------------------------------------------------------------------------*/

uint16_t SerialModbusBase::usReplyWord( size_t xNbr, size_t xOffset )
{
    return ( ( uint16_t ) ucReplyByte( xNbr, xOffset + xNbr ) << 8 ) | ucReplyByte( xNbr + 1, xOffset + xNbr );
}
/*----------------------------------------------------------------------------*/

uint32_t SerialModbusBase::ulReplyDword( size_t xNbr, size_t xOffset )
{
    return ( ( uint32_t ) usReplyWord( xNbr, xOffset ) << 16 ) | usReplyWord( xNbr + 2, xOffset );
}
/*----------------------------------------------------------------------------*/

uint64_t SerialModbusBase::uxReplyQword( size_t xNbr, size_t xOffset )
{
    return ( ( uint64_t ) ulReplyDword( xNbr, xOffset ) << 32 ) | ulReplyDword( xNbr + 4, xOffset );
}
/*----------------------------------------------------------------------------*/

#if( configMB_PROCESS_LOOP_HOOK == 1 )

    void SerialModbusBase::setProcessLoopHook( void (* loopHookFunction)( void ) )
    {
        /* INFO: No NULL check to provide the ability to turn the loop hook 
         * function mechanism on and off. */
        vProcessLoopHook = loopHookFunction;
    }

#endif
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusBase::xSetException( MB_Exception_t xException )
{
    xStatus = xException;
    return xStatus;
}
/*----------------------------------------------------------------------------*/

void SerialModbusBase::vStartInterFrameDelay( void )
{
    ulTimerInterFrameDelayUs = micros();
}
/*----------------------------------------------------------------------------*/

void SerialModbusBase::vStartInterCharacterTimeout( void )
{
    ulTimerInterCharacterTimeoutUs = micros();
}
/*----------------------------------------------------------------------------*/

bool SerialModbusBase::bTimeoutInterFrameDelay( void )
{
    return ( micros() - ulTimerInterFrameDelayUs ) >= ulInterFrameDelayUs;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusBase::bTimeoutInterCharacterTimeout( void )
{
    return ( micros() - ulTimerInterCharacterTimeoutUs ) >= ulInterCharacterTimeoutUs;
}
/*----------------------------------------------------------------------------*/

uint32_t SerialModbusBase::getInterCharacterTimeout( void )
{
    return ulInterCharacterTimeoutUs;
}
/*----------------------------------------------------------------------------*/

uint32_t SerialModbusBase::getInterFrameDelay( void )
{
    return ulInterFrameDelayUs;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusBase::setInterCharacterTimeout( uint32_t timeUs )
{
    if( timeUs != 0 )
    {
        ulInterCharacterTimeoutUs = timeUs;
        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusBase::setInterFrameDelay( uint32_t timeUs )
{
    if( timeUs != 0 )
    {
        ulInterFrameDelayUs = timeUs;
        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusBase::bReceiveByte( uint8_t * pucReceiveBuffer, size_t * pxBufferLength )
{
    if( ( pucReceiveBuffer != NULL ) && ( pxBufferLength != NULL ) )
    {
        if( pxSerial != NULL )
        {
            if( pxSerial->available() > 0 )
            {
                pucReceiveBuffer[ (*pxBufferLength)++ ] = pxSerial->read();

                return true;
            }
        }
#if defined( configMB_SERIAL_SW )
        else if( pxSWSerial != NULL )
        {
            if( pxSWSerial->available() > 0 )
            {
                pucReceiveBuffer[ (*pxBufferLength)++ ] = pxSWSerial->read();

                return true;
            }
        }
#endif
    }

    return false;
}
/*----------------------------------------------------------------------------*/

size_t SerialModbusBase::xSendData( uint8_t * pucSendBuffer, size_t pxBufferLength )
{
    size_t xBytesSent = 0;

    if( ( pucSendBuffer != NULL ) && ( pxBufferLength > 0 ) )
    {
        if( vSerialCtrlTx != NULL )
        {
            ( vSerialCtrlTx )();
        }

        if( pxSerial != NULL )
        {
            xBytesSent = pxSerial->write( pucSendBuffer, pxBufferLength );
            pxSerial->flush();
        }
#if defined( configMB_SERIAL_SW )
        else if( pxSWSerial != NULL )
        {
            xBytesSent = pxSWSerial->write( pucSendBuffer, pxBufferLength );
        }
#endif
        #if( configMB_MODE == configMB_MODE_RTU )
        {
            /* Wait the amount of microseconds for the Inter Frame Delay to let
             * the receiving device detect the end of the frame. */
            vDelayUs( ulInterFrameDelayUs );
        }
        #endif

        if( vSerialCtrlRx != NULL )
        {
            ( vSerialCtrlRx )();
        }
    }

    return xBytesSent;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusBase::bCalculateTimeouts( uint32_t ulBaud, uint32_t ulConfig )
{
    uint32_t ulNbrOfBits = 0;

    if( ulBaud > 19200 )
    {
        /* For transfer rates higer than 19200 Baud Modbus recommend fixed
         * values for the inter character timeout and the inter frame delay. */
        ulInterCharacterTimeoutUs =  750;
        ulInterFrameDelayUs       = 1750;
    }
    else
    {
        /* For transfer rates lower than 19200 Baud the timing is more critical
         * and must be calculated.
         *
         * Calculation example for the inter character timeout with 19200 baud
         * (and standard Modbus UART configuration SERIAL_8N1 - 11 Bits):
         *
         *     19200 Baud / 11 Bits = 1745.45'
         *
         * That means 1745.45' characters could be transmitted in 1 Second.
         *
         *     1000 ms / 1745.45 = 0.5729167' ms
         *
         * So, one character needs 0.5729167' ms to be transmitted. The inter
         * character timeout is defined as 1.5x the time needed to transfer one
         * character (Inter frame delay is 3.5x).
         *
         *     0.5729167' ms * 1.5 = 0.859375 ms
         *
         * And because our timers are working with microseconds we have a final
         * value of 859.375 us.
         *
         *     0.859375 ms * 1000 = 859.375 us
         */

        switch( ulConfig )
        {
            case SERIAL_5N1 :
            case SERIAL_5E1 :
            case SERIAL_5O1 :
            {
                ulNbrOfBits = 8;
                break;
            }
            case SERIAL_6N1 :
            case SERIAL_5N2 :
            case SERIAL_6E1 :
            case SERIAL_5E2 :
            case SERIAL_6O1 :
            case SERIAL_5O2 :
            {
                ulNbrOfBits = 9;
                break;
            }
            case SERIAL_7N1 :
            case SERIAL_6N2 :
            case SERIAL_7E1 :
            case SERIAL_6E2 :
            case SERIAL_7O1 :
            case SERIAL_6O2 :
            {
                ulNbrOfBits = 10;
                break;
            }
            case SERIAL_8N1 :
            case SERIAL_7N2 :
            case SERIAL_8E1 :
            case SERIAL_7E2 :
            case SERIAL_8O1 :
            case SERIAL_7O2 :
            {
                ulNbrOfBits = 11;
                break;
            }
            case SERIAL_8N2 :
            case SERIAL_8E2 :
            case SERIAL_8O2 :
            {
                ulNbrOfBits = 12;
                break;
            }
            default :
            {
                return false;
            }
        }

        /* Basic formula:
         *
         *               1000
         *     timeout = ---- * nbrOfBits * characterTimes * 1000
         *               baud
         *
         * Final formulas:
         *
         *                             nbrOfBits * 1000000 * 1.5
         *     interCharacterTimeout = -------------------------
         *                                     baud
         *
         *                       nbrOfBits * 1000000 * 3.5
         *     interFrameDelay = -------------------------
         *                                 baud
         */

        ulInterCharacterTimeoutUs = ulNbrOfBits * ( 1500000 / ulBaud );
        ulInterFrameDelayUs       = ulNbrOfBits * ( 3500000 / ulBaud );
    }

    return true;
}
/*----------------------------------------------------------------------------*/

void SerialModbusBase::vDelayUs( uint32_t ulDelayUs )
{
    if( vCustomDelayUs == NULL )
    {
        delayMicroseconds( ulDelayUs );
    }
    else
    {
        ( vCustomDelayUs )( ulDelayUs );
    }
}
/*----------------------------------------------------------------------------*/

void SerialModbusBase::setCustomDelay( void (* customDelay)( uint32_t delayUs ) )
{
    if( customDelay != NULL )
    {
        vCustomDelayUs = customDelay;
    }
}
/*----------------------------------------------------------------------------*/

const char * SerialModbusBase::getExceptionString( MB_Exception_t exception )
{
    for( size_t i = 0; pxExceptionStrings[ i ].pcString != NULL; i++ )
    {
        if( pxExceptionStrings[ i ].xException == exception )
        {
            return pxExceptionStrings[ i ].pcString;
        }
    }

    return NULL;
}
