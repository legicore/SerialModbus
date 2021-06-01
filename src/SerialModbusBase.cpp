////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusBase.cpp
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

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusConfig.h"
#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"

#include <Arduino.h>
#if defined( COMPAT_SOFTWARE_SERIAL )
    #include <SoftwareSerial.h>
#endif

/*-----------------------------------------------------------*/

typedef struct MBExceptionString_s
{
    uint8_t exceptionCode;
    const char * exceptionString;
}
MBExceptionString_t;

static const MBExceptionString_t pxExceptionStrings[] = {

    /* Standard exception codes. */
    { ILLEGAL_FUNCTION,                        "ILLEGAL_FUNCTION" },
    { ILLEGAL_DATA_ADDRESS,                    "ILLEGAL_DATA_ADDRESS" },
    { ILLEGAL_DATA_VALUE,                      "ILLEGAL_DATA_VALUE" },
    { SLAVE_DEVICE_FAILURE,                    "SLAVE_DEVICE_FAILURE" },
    { ACKNOWLEDGE,                             "ACKNOWLEDGE" },
    { SLAVE_DEVICE_BUSY,                       "SLAVE_DEVICE_BUSY" },
    { MEMORY_PARITY_ERROR,                     "MEMORY_PARITY_ERROR" },
    { GATEWAY_PATH_UNAVAILABLE,                "GATEWAY_PATH_UNAVAILABLE" },
    { GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND, "GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND" },

    /* Non-standard exception codes. */
    { ILLEGAL_REQUEST,                         "ILLEGAL_REQUEST" },
    { CHARACTER_OVERRUN,                       "CHARACTER_OVERRUN" },
    { NO_REPLY,                                "NO_REPLY" },
    { ILLEGAL_CHECKSUM,                        "ILLEGAL_CHECKSUM" },
    { ILLEGAL_STATE,                           "ILLEGAL_STATE" },
    { ILLEGAL_BYTE_COUNT,                      "ILLEGAL_BYTE_COUNT" },
    { ILLEGAL_COIL_VALUE,                      "ILLEGAL_COIL_VALUE" },
    { ILLEGAL_OUTPUT_ADDRESS,                  "ILLEGAL_OUTPUT_ADDRESS" },
    { ILLEGAL_OUTPUT_VALUE,                    "ILLEGAL_OUTPUT_VALUE" },
    { ILLEGAL_QUANTITY,                        "ILLEGAL_QUANTITY" },
    { ILLEGAL_QUERY_DATA,                      "ILLEGAL_QUERY_DATA" },
    { ILLEGAL_SUB_FUNCTION,                    "ILLEGAL_SUB_FUNCTION" },
    { ILLEGAL_REPLY_SUB_FUNCTION,              "ILLEGAL_REPLY_SUB_FUNCTION" },

#if( configEXTENDED_EXCEPTION_CODES == 1 )

    /* Extended exception codes for slave replies. */
    { SLV_ILLEGAL_FUNCTION,                    "SLV_ILLEGAL_FUNCTION" },
    { SLV_ILLEGAL_STATE,                       "SLV_ILLEGAL_STATE" },
    { SLV_ILLEGAL_DATA_ADDRESS,                "SLV_ILLEGAL_DATA_ADDRESS" },
    { SLV_ILLEGAL_ACCESS,                      "SLV_ILLEGAL_ACCESS" },
    { SLV_ILLEGAL_QUANTITY,                    "SLV_ILLEGAL_QUANTITY" },
    { SLV_ILLEGAL_COIL_VALUE,                  "SLV_ILLEGAL_COIL_VALUE" },
    { SLV_ILLEGAL_INPUT_DELIMITER,             "SLV_ILLEGAL_INPUT_DELIMITER" },
    { SLV_ILLEGAL_SUB_FUNCTION,                "SLV_ILLEGAL_SUB_FUNCTION" },
    { SLV_ILLEGAL_DATA_VALUE,                  "SLV_ILLEGAL_DATA_VALUE" },

#endif

    { OK,  "OK" },
    { NOK, "NOK" },

    /* Marks the end of the list. */
    { 0x00, NULL }
};
/*-----------------------------------------------------------*/

SerialModbusBase::SerialModbusBase()
{
    xRequestLength = 0;
    xReplyLength = 0;

    pxSerial = NULL;
    ulSerialConfig = 0;
    #if defined( COMPAT_SOFTWARE_SERIAL )
    {
        pxSerialSoftware = NULL;
        ulSerialConfig = SERIAL_CONFIG_DEFAULT;
    }
    #endif

    vSerialCtrlTx = NULL;
    vSerialCtrlRx = NULL;

    xException = OK;

    #if( configMODE == configMODE_RTU )
    {
        ulInterFrameDelayUs = 0;
        ulTimerInterFrameDelayUs = 0;
        ulInterCharacterTimeoutUs = 0;
        ulTimerInterCharacterTimeoutUs = 0;
    }
    #endif

    #if( configPROCESS_LOOP_HOOK == 1 )
    {
        vProcessLoopHook = NULL;
    }
    #endif

    cAsciiInputDelimiter = configASCII_INPUT_DELIMITER;

    vCustomDelayUs = NULL;

    #if( configMODE == configMODE_RTU )
    {
        xChecksumLength = 2;
    }
    #endif
    #if( configMODE == configMODE_ASCII )
    {
        xChecksumLength = 1;
    }
    #endif
}
/*-----------------------------------------------------------*/

bool SerialModbusBase::begin( uint32_t baud, Serial_t * serial, uint32_t config )
{
    if( baud == 0 || serial == NULL || config == 0 )
    {
        return false;
    }

    pxSerial = serial;
    ulSerialConfig = config;
    pxSerial->begin( baud, config );

    #if( configMODE == configMODE_RTU )
    {
        if( bCalculateTimeouts( baud ) != true )
        {
            return false;
        }
    }
    #endif

    return true;
}
/*-----------------------------------------------------------*/

#if defined( COMPAT_SOFTWARE_SERIAL )

    bool SerialModbusBase::begin( uint32_t baud, SoftwareSerial * serial )
    {
        if( baud == 0 || serial == NULL )
        {
            return false;
        }

        pxSerialSoftware = serial;
        pxSerialSoftware->begin( baud );

        #if( configMODE == configMODE_RTU )
        {
            if( bCalculateTimeouts( baud ) != true )
            {
                return false;
            }
        }
        #endif

        return true;
    }

#endif
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusBase::xSetChecksum( uint8_t * pucFrame, size_t * pxFrameLength )
{
    uint16_t usTempChecksum;

    if( *pxFrameLength == 0 )
    {
        return NOK;
    }

    #if( configMODE == configMODE_RTU )
    {
        usTempChecksum = usCRC16( pucFrame, *pxFrameLength );
        pucFrame[ (*pxFrameLength)++ ] =  lowByte( usTempChecksum );
        pucFrame[ (*pxFrameLength)++ ] = highByte( usTempChecksum );
    }
    #endif

    #if( configMODE == configMODE_ASCII )
    {
        usTempChecksum = ( uint16_t ) ucLRC( pucFrame, *pxFrameLength );
        pucFrame[ (*pxFrameLength)++ ] = ( uint8_t ) usTempChecksum;
    }
    #endif

    return OK;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusBase::xCheckChecksum( uint8_t * pucFrame, size_t xFrameLength )
{
    uint16_t usTempChecksum;

    if( xFrameLength == 0 )
    {
        return NOK;
    }

    #if( configMODE == configMODE_RTU )
    {
        usTempChecksum = usCRC16( pucFrame, xFrameLength - 2 );
        if( pucFrame[ xFrameLength - 2 ] == lowByte( usTempChecksum ) )
        {
            if( pucFrame[ xFrameLength - 1 ] == highByte( usTempChecksum ) )
            {
                return OK;
            }
        }
    }
    #endif

    #if( configMODE == configMODE_ASCII )
    {
        usTempChecksum = ( uint16_t ) ucLRC( pucFrame, xFrameLength - 1 );
        if( pucFrame[ xFrameLength - 1 ] == ( uint8_t ) usTempChecksum )
        {
            return OK;
        }
    }
    #endif

    return NOK;
}
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

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

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_ASCII )

    uint8_t SerialModbusBase::ucLRC( uint8_t * pucData, size_t xDataLength )
    {
        uint8_t ucLrc = 0;

        while( xDataLength-- > 0 )
        {
            ucLrc += *pucData++;
        }

        return ( uint8_t ) ( -( ( int8_t ) ucLrc ) );
    }

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_ASCII )

    MBStatus_t SerialModbusBase::xRtuToAscii( uint8_t * pucFrame, size_t * pxFrameLength )
    {
        if( *pxFrameLength == 0 )
        {
            return NOK;
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

        return OK;
    }

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_ASCII )

    MBStatus_t SerialModbusBase::xAsciiToRtu( uint8_t * pucFrame, size_t * pxFrameLength )
    {
        if( *pxFrameLength == 0 )
        {
            return NOK;
        }

        *pxFrameLength = ( *pxFrameLength - 3 ) / 2;

        for( size_t i = 0; i < *pxFrameLength; i++ )
        {
            pucFrame[ i ] = ucAsciiToByte( pucFrame[ ( i * 2 ) + 1 ], pucFrame[ ( i * 2 ) + 2 ] );
        }

        return OK;
    }

#endif
/*-----------------------------------------------------------*/

void SerialModbusBase::vClearReplyFrame( void )
{
    //memset( pucReplyFrame, 0x00, xReplyLength );
    xReplyLength = 0;
}
/*-----------------------------------------------------------*/

void SerialModbusBase::vClearRequestFrame( void )
{
    //memset( pucRequestFrame, 0x00, xRequestLength );
    xRequestLength = 0;
}
/*-----------------------------------------------------------*/

void SerialModbusBase::setSerialCtrl( void (*serialCtrlTx)( void ), void (*serialCtrlRx)( void ) )
{
    vSerialCtrlTx = serialCtrlTx;
    vSerialCtrlRx = serialCtrlRx;
}
/*-----------------------------------------------------------*/

uint8_t SerialModbusBase::ucRequestByte( size_t xNbr, size_t xOffset )
{
    return pucRequestFrame[ xNbr + xOffset ];
}
/*-----------------------------------------------------------*/

uint16_t SerialModbusBase::usRequestWord( size_t xNbr, size_t xOffset )
{
    return ( ( uint16_t ) ucRequestByte( xNbr, xOffset + xNbr ) << 8 ) | ucRequestByte( xNbr + 1, xOffset + xNbr );
}
/*-----------------------------------------------------------*/

uint32_t SerialModbusBase::ulRequestDword( size_t xNbr, size_t xOffset )
{
    return ( ( uint32_t ) usRequestWord( xNbr, xOffset ) << 16 ) | usRequestWord( xNbr + 2, xOffset );
}
/*-----------------------------------------------------------*/

uint64_t SerialModbusBase::uxRequestQword( size_t xNbr, size_t xOffset )
{
    return ( ( uint64_t ) ulRequestDword( xNbr, xOffset ) << 32 ) | ulRequestDword( xNbr + 4, xOffset );
}
/*-----------------------------------------------------------*/

uint8_t SerialModbusBase::ucReplyByte( size_t xNbr, size_t xOffset )
{
    return pucReplyFrame[ xNbr + xOffset ];
}
/*-----------------------------------------------------------*/

uint16_t SerialModbusBase::usReplyWord( size_t xNbr, size_t xOffset )
{
    return ( ( uint16_t ) ucReplyByte( xNbr, xOffset + xNbr ) << 8 ) | ucReplyByte( xNbr + 1, xOffset + xNbr );
}
/*-----------------------------------------------------------*/

uint32_t SerialModbusBase::ulReplyDword( size_t xNbr, size_t xOffset )
{
    return ( ( uint32_t ) usReplyWord( xNbr, xOffset ) << 16 ) | usReplyWord( xNbr + 2, xOffset );
}
/*-----------------------------------------------------------*/

uint64_t SerialModbusBase::uxReplyQword( size_t xNbr, size_t xOffset )
{
    return ( ( uint64_t ) ulReplyDword( xNbr, xOffset ) << 32 ) | ulReplyDword( xNbr + 4, xOffset );
}
/*-----------------------------------------------------------*/

#if( configPROCESS_LOOP_HOOK == 1 )

    void SerialModbusBase::setProcessLoopHook( void (*loopHookFunction)( void ) )
    {
        if( loopHookFunction != NULL )
        {
            vProcessLoopHook = loopHookFunction;
        }
        else
        {
            vProcessLoopHook = NULL;
        }
    }

#endif
/*-----------------------------------------------------------*/

MBException_t SerialModbusBase::xSetException( MBException_t xExceptionPar )
{
    xException = xExceptionPar;
    return xException;
}
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

    void SerialModbusBase::vStartInterFrameDelay( void )
    {
        ulTimerInterFrameDelayUs = micros();
    }

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

    void SerialModbusBase::vStartInterCharacterTimeout( void )
    {
        ulTimerInterCharacterTimeoutUs = micros();
    }

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

    bool SerialModbusBase::bTimeoutInterFrameDelay( void ) const
    {
        return ( micros() - ulTimerInterFrameDelayUs ) >= ulInterFrameDelayUs;
    }

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

    bool SerialModbusBase::bTimeoutInterCharacterTimeout( void ) const
    {
        return ( micros() - ulTimerInterCharacterTimeoutUs ) >= ulInterCharacterTimeoutUs;
    }

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_ASCII )

    uint8_t SerialModbusBase::ucByteToAsciiHi( uint8_t ucByte )
    {
        return ( ucByte >> 4 ) + ( ( ( ucByte >> 4 ) < 10 ) ? 48 : 55 );
    }

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_ASCII )

    uint8_t SerialModbusBase::ucByteToAsciiLo( uint8_t ucByte )
    {
        return ( ucByte & 0x0F ) + ( ( ( ucByte & 0x0F ) < 10 ) ? 48 : 55 );
    }

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_ASCII )

    uint8_t SerialModbusBase::ucAsciiToByte( uint8_t ucAsciiHi, uint8_t ucAsciiLo )
    {
        return ( ( ucAsciiHi - ( ( ucAsciiHi < 64 ) ? 48 : 55 ) ) << 4 ) | ( ucAsciiLo - ( ( ucAsciiLo < 64 ) ? 48 : 55 ) );
    }

#endif
/*-----------------------------------------------------------*/

bool SerialModbusBase::bReceiveByte( uint8_t * pucReceiveBuffer, size_t * pxBufferLength )
{
    if( pxSerial != NULL )
    {
        if( pxSerial->available() > 0 )
        {
            pucReceiveBuffer[ (*pxBufferLength)++ ] = pxSerial->read();

            return true;
        }
    }
#if defined( COMPAT_SOFTWARE_SERIAL )
    else if( pxSerialSoftware != NULL )
    {
        if( pxSerialSoftware->available() > 0 )
        {
            pucReceiveBuffer[ (*pxBufferLength)++ ] = pxSerialSoftware->read();

            return true;
        }
    }
#endif

    return false ;
}
/*-----------------------------------------------------------*/

void SerialModbusBase::vSendData( uint8_t * pucSendBuffer, size_t pxBufferLength )
{
    if( vSerialCtrlTx != NULL )
    {
        (*vSerialCtrlTx)();
    }

    if( pxSerial != NULL )
    {
        pxSerial->write( pucSendBuffer, pxBufferLength );
        pxSerial->flush();
    }
#if defined( COMPAT_SOFTWARE_SERIAL )
    else if( pxSerialSoftware != NULL )
    {
        pxSerialSoftware->write( pucSendBuffer, pxBufferLength );
    }
#endif

    #if( configMODE == configMODE_RTU )
    {
        /* Wait the amount of microseconds for the Inter Frame Delay to let the
        receiving device detect the end of the frame. */
        vDelayUs( ulInterFrameDelayUs );
    }
    #endif

    if( vSerialCtrlRx != NULL )
    {
        (*vSerialCtrlRx)();
    }
}
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

    bool SerialModbusBase::bCalculateTimeouts( uint32_t ulBaud )
    {
        uint8_t ucNbrOfBits = 0;

        if( ulBaud > 19200 )
        {
            /* For transfer rates higer than 19200 Baud Modbus recommend fixed
            values for the inter character timeout and the inter frame delay. */
            ulInterCharacterTimeoutUs = 750;
            ulInterFrameDelayUs = 1750;
        }
        else
        {
            /* For transfer rates lower than 19200 Baud the timing is more
            critical and must be calculated. */

            /* Calculation example for the inter character timeout with 19200
            Baud (and standard Modbus UART configuration SERIAL_8N1 - 11 Bits):

                19200 Baud / 11 Bits = 1745.45'

            That means 1745.45' characters could be transmitted in 1 Second.

                1000 ms / 1745.45 = 0.5729167' ms

            So one character needs 0.5729167' ms to be transmitted.
            The inter character timeout is defined as 1.5x the time needed to
            transfer one character (Inter frame delay is 3.5x).

                0.5729167' ms * 1.5 = 0.859375 ms

            And because our timers are working with microseconds we have a final
            value of 859.375 us.

                0.859375 ms * 1000 = 859.375 us */

            switch( ulSerialConfig )
            {
                case SERIAL_5N1 :
                case SERIAL_5E1 :
                case SERIAL_5O1 :
                {
                    ucNbrOfBits = 8;
                    break;
                }
                case SERIAL_6N1 :
                case SERIAL_5N2 :
                case SERIAL_6E1 :
                case SERIAL_5E2 :
                case SERIAL_6O1 :
                case SERIAL_5O2 :
                {
                    ucNbrOfBits = 9;
                    break;
                }
                case SERIAL_7N1 :
                case SERIAL_6N2 :
                case SERIAL_7E1 :
                case SERIAL_6E2 :
                case SERIAL_7O1 :
                case SERIAL_6O2 :
                {
                    ucNbrOfBits = 10;
                    break;
                }
                case SERIAL_8N1 :
                case SERIAL_7N2 :
                case SERIAL_8E1 :
                case SERIAL_7E2 :
                case SERIAL_8O1 :
                case SERIAL_7O2 :
                {
                    ucNbrOfBits = 11;
                    break;
                }
                case SERIAL_8N2 :
                case SERIAL_8E2 :
                case SERIAL_8O2 :
                {
                    ucNbrOfBits = 12;
                    break;
                }
                default :
                {
                    return false;
                }
            }

            /* General formula:

                          1000
                timeout = ---- * nbrOfBits * characterTimes * 1000
                          baud

            Final formulas:

                                        nbrOfBits * 1000000 * 1.5
                interCharacterTimeout = -------------------------
                                                baud

                                  nbrOfBits * 1000000 * 3.5
                interFrameDelay = -------------------------
                                            baud */

            ulInterCharacterTimeoutUs = ( ucNbrOfBits * 1500000 ) / ulBaud;
            ulInterFrameDelayUs       = ( ucNbrOfBits * 3500000 ) / ulBaud;
        }

        return true;
    }

#endif
/*-----------------------------------------------------------*/

void SerialModbusBase::vDelayUs( uint32_t ulDelayUs )
{
    if( vCustomDelayUs == NULL )
    {
        delayMicroseconds( ulDelayUs );
    }
    else
    {
        (*vCustomDelayUs)( ulDelayUs );
    }
}
/*-----------------------------------------------------------*/

void SerialModbusBase::setCustomDelay( void (*customDelay)( uint32_t delayUs ) )
{
    vCustomDelayUs = customDelay;
}
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

	uint32_t SerialModbusBase::getInterCharacterTimeout( void ) const
	{
		return ulInterCharacterTimeoutUs;
	}

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

	uint32_t SerialModbusBase::getInterFrameDelay( void ) const
	{
		return ulInterFrameDelayUs;
	}

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

	int8_t SerialModbusBase::setInterCharacterTimeout( uint32_t timeUs )
	{
		if( timeUs != 0 )
		{
			ulInterCharacterTimeoutUs = timeUs;
			return 0;
		}

		return -1;
	}

#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )

	int8_t SerialModbusBase::setInterFrameDelay( uint32_t timeUs )
	{
		if( timeUs != 0 )
		{
			ulInterFrameDelayUs = timeUs;
			return 0;
		}

		return -1;
	}

#endif
/*-----------------------------------------------------------*/

const char * SerialModbusBase::getExceptionString( uint8_t exceptionCode )
{
    for( size_t i = 0; pxExceptionStrings[ i ].exceptionString != NULL; i++ )
    {
        if( pxExceptionStrings[ i ].exceptionCode == exceptionCode )
        {
            return pxExceptionStrings[ i ].exceptionString;
        }
    }

    return "__NO_STRING__";
}
