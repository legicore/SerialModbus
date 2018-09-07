///////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusBase.cpp
 *
 * @author      legicore
 *
 * @brief       xxx
 */
///////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

#include "SerialModbusConfig.h"
#include "SerialModbusBase.h"

/*-----------------------------------------------------------*/

SerialModbusBase::SerialModbusBase()
{
    xRequestLength = 0;
    xReplyLength   = 0;

    pxSerial = NULL;
    pxSerialSoftware = NULL;

    setSerialCtrl( NULL, NULL );
    
    xSetException( OK );

    setDataList( NULL );

    #if( configMODE == configMODE_RTU )
    {
        ulInterFrameDelayUs = configINTER_FRAME_DELAY_US;
        ulTimerInterFrameDelayUs = 0;
        ulInterCharacterTimeoutUs = configINTER_CHARACTER_TIMEOUT_US;
        ulTimerInterCharacterTimeoutUs = 0;
    }
    #endif
    
    #if( configPROCESS_LOOP_HOOK == 1 )
    {
        setProcessLoopHook( NULL );
    }
    #endif

    #if( configMODE == configMODE_ASCII )
    {
        cAsciiInputDelimiter = configASCII_INPUT_DELIMITER;
    }
    #endif
}
/*-----------------------------------------------------------*/

void SerialModbusBase::begin( uint32_t baud, HardwareSerial * serial, uint8_t config )
{
	pxSerial = serial;
    pxSerial->begin( baud, config );
}
/*-----------------------------------------------------------*/

void SerialModbusBase::begin( uint32_t baud, SoftwareSerial * serial )
{
	pxSerialSoftware = serial;
    pxSerialSoftware->begin( baud );
}
/*-----------------------------------------------------------*/

void SerialModbusBase::setDataList( const MBData_t * dataList )
{
    if( dataList != NULL )
    {
        pxDataList = ( MBData_t * ) dataList;
    }
    else
    {
        pxDataList = NULL;
    }
    
    xDataListIndex = 0;
}
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )
void SerialModbusBase::setInterFrameDelay( uint32_t timeMs )
{
    ulInterFrameDelayUs = timeMs * 1000;
}
#endif
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusBase::xSetChecksum( uint8_t * pucFrame, size_t * pxFrameLength )
{
    uint16_t usTempChecksum;

    if( ( pucFrame == NULL ) || ( pxFrameLength == NULL ) )
    {
        return NOK_NULL_POINTER;
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

    if( pucFrame == NULL )
    {
        return NOK_NULL_POINTER;
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
                usCrc >>= 1;
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
    if( ( pucFrame == NULL ) || ( pxFrameLength == NULL ) )
    {
        return NOK_NULL_POINTER;
    }
    
    pucFrame[ ( *pxFrameLength * 2 ) + 2 ] = cAsciiInputDelimiter;
    pucFrame[ ( *pxFrameLength * 2 ) + 1 ] = '\r';

    for( size_t i = *pxFrameLength - 1; i >= 0; i-- )
    {
        pucFrame[ ( i * 2 ) + 2 ] = ucByteToAsciiLo( pucFrame[ i ] );
        pucFrame[ ( i * 2 ) + 1 ] = ucByteToAsciiHi( pucFrame[ i ] );
    }

    pucFrame[ 0 ] = ':';

    *pxFrameLength = ( *pxFrameLength * 2 ) + 3;
    
    return OK;
}
#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_ASCII )
MBStatus_t SerialModbusBase::xAsciiToRtu( uint8_t * pucFrame, size_t * pxFrameLength )
{
    if( ( pucFrame == NULL ) || ( pxFrameLength == NULL ) )
    {
        return NOK_NULL_POINTER;
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
bool SerialModbusBase::bTimeoutInterFrameDelay( void )
{
    return ( micros() - ulTimerInterFrameDelayUs ) > ulInterFrameDelayUs;
}
#endif
/*-----------------------------------------------------------*/

#if( configMODE == configMODE_RTU )
bool SerialModbusBase::bTimeoutInterCharacterTimeout( void )
{
    return ( micros() - ulTimerInterCharacterTimeoutUs ) > ulInterCharacterTimeoutUs;
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
    else if( pxSerialSoftware != NULL )
    {
        if( pxSerialSoftware->available() > 0 )
        {
            pucReceiveBuffer[ (*pxBufferLength)++ ] = pxSerialSoftware->read();

            return true;
        }
    }

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
    else if( pxSerialSoftware != NULL )
    {
        pxSerialSoftware->write( pucSendBuffer, pxBufferLength );
    }

    if( vSerialCtrlRx != NULL )
    {
        (*vSerialCtrlRx)();
    }
}
