////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusClient.cpp
 * 
 * @author      Martin Legleiter
 * 
 * @brief       TODO
 * 
 * @copyright   (c) 2023 Martin Legleiter
 * 
 * @license     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include "SerialModbusConfig.h"
#include "SerialModbusBase.h"
#include "SerialModbusClient.h"

#include <Arduino.h>

/*-----------------------------------------------------------*/

SerialModbusClient::SerialModbusClient()
{
    xState = CLIENT_IDLE;

    pxRequestMap = NULL;
    xRequestMapIndex = 0;
    pxRequest = NULL;
    bSkipRequestMap = false;

    ulTurnaroundDelayMs = configTURNAROUND_DELAY_MS;
    ulResponseTimeoutMs = configRESPONSE_TIMEOUT_MS;

    ulTimerResponseTimeoutMs = 0;
    ulTimerTurnaroundDelayMs = 0;

    xStatusSimpleAPI = OK;
}
/*-----------------------------------------------------------*/

void SerialModbusClient::vSetState( MBClientState_t xStatePar )
{
    xState = xStatePar;
}
/*-----------------------------------------------------------*/

void SerialModbusClient::vStartTurnaroundDelay( void )
{
    ulTimerTurnaroundDelayMs = millis();
}
/*-----------------------------------------------------------*/

void SerialModbusClient::vStartResponseTimeout( void )
{
    ulTimerResponseTimeoutMs = millis();
}
/*-----------------------------------------------------------*/

bool SerialModbusClient::bTimeoutTurnaroundDelay( void )
{
    return ( millis() - ulTimerTurnaroundDelayMs ) >= ulTurnaroundDelayMs;
}
/*-----------------------------------------------------------*/

bool SerialModbusClient::bTimeoutResponseTimeout( void )
{
    return ( millis() - ulTimerResponseTimeoutMs ) >= ulResponseTimeoutMs;
}
/*-----------------------------------------------------------*/

uint32_t SerialModbusClient::getResponseTimeout( void )
{
    return ulResponseTimeoutMs;
}
/*-----------------------------------------------------------*/

uint32_t SerialModbusClient::getTurnaroundDelay( void )
{
    return ulTurnaroundDelayMs;
}
/*-----------------------------------------------------------*/

bool SerialModbusClient::setResponseTimeout( uint32_t timeMs )
{
    if( timeMs != 0 )
    {
        ulResponseTimeoutMs = timeMs;
        return true;
    }

    return false;
}
/*-----------------------------------------------------------*/

bool SerialModbusClient::setTurnaroundDelay( uint32_t timeMs )
{
    if( timeMs != 0 )
    {
        ulTurnaroundDelayMs = timeMs;
        return true;
    }

    return false;
}
/*-----------------------------------------------------------*/

void SerialModbusClient::setRequestMap( const MBRequest_t * requestMap )
{
    pxRequestMap = requestMap;
    xRequestMapIndex = 0;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusClient::xProcessRequestMap( void )
{
    if( pxRequestMap != NULL )
    {
        if( pxRequestMap[ xRequestMapIndex ].functionCode == 0x00 )
        {
            xRequestMapIndex = 0;
        }

        return setRequest( &pxRequestMap[ xRequestMapIndex++ ], true );
    }
    else
    {
        xRequestMapIndex = 0;
    }

    return OK;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusClient::setRequest( const MBRequest_t * request, bool requestMap )
{
    if( request == NULL )
    {
        return xSetException( ILLEGAL_REQUEST );
    }

    if( requestMap == false )
    {
        bSkipRequestMap = true;
    }

    ( void ) xSetException( OK );

    if( ( request->id > configID_SERVER_MAX ) ||
        ( request->functionCode == 0x00 ) ||
        ( request->dataSize == 0 ) )
    {
        return xSetException( ILLEGAL_REQUEST );
    }

    vClearRequestFrame();

    pucRequestFrame[ 0 ] = request->id;
    pucRequestFrame[ 1 ] = request->functionCode;
    pucRequestFrame[ 2 ] = highByte( request->address );
    pucRequestFrame[ 3 ] =  lowByte( request->address );

    switch( request->functionCode )
    {
#if( ( configFC03 == 1 ) || ( configFC04 == 1 ) )
        case READ_HOLDING_REGISTERS:
        case READ_INPUT_REGISTERS:
        {
            pucRequestFrame[ 4 ] = highByte( request->dataSize );
            pucRequestFrame[ 5 ] =  lowByte( request->dataSize );
            xRequestLength = 6;

            break;
        }
#endif
#if( ( configFC05 == 1 ) || ( configFC06 == 1 ) )
        case WRITE_SINGLE_COIL:
        case WRITE_SINGLE_REGISTER:
        {
            if( request->data != NULL )
            {
                pucRequestFrame[ 4 ] = highByte( request->data[ 0 ] );
                pucRequestFrame[ 5 ] =  lowByte( request->data[ 0 ] );
                xRequestLength = 6;
            }
            else
            {
                return xSetException( ILLEGAL_REQUEST );
            }

            break;
        }
#endif
#if( configFC08 == 1 )
        case DIAGNOSTIC:
        {
            xRequestLength = 6;

            switch( request->address )
            {
#if( configSFC00 == 1 )
                case RETURN_QUERY_DATA:
                {
                    if( request->data != NULL )
                    {
                        xRequestLength = 4;

                        for( size_t i = 0; i < request->dataSize; i++ )
                        {
                            pucRequestFrame[ 4 + i ] = ( uint8_t ) ( ( char * ) request->data )[ i ];
                            xRequestLength++;
                        }
                    }
                    else
                    {
                        return xSetException( ILLEGAL_REQUEST );
                    }

                    break;
                }
#endif
#if( configSFC01 == 1 )
                case RESTART_COMMUNICATIONS_OPTION:
                {
                    if( request->data != NULL )
                    {
                        pucRequestFrame[ 4 ] = highByte( request->data[ 0 ] );
                        pucRequestFrame[ 5 ] =  lowByte( request->data[ 0 ] );
                    }
                    else
                    {
                        return xSetException( ILLEGAL_REQUEST );
                    }

                    break;
                }
#endif
#if( configSFC03 == 1 )
                case CHANGE_ASCII_INPUT_DELIMITER:
                {
                    if( isAscii( *( ( char * ) request->data ) ) == true )
                    {
                        pucRequestFrame[ 4 ] = ( uint8_t ) *( ( char * ) request->data );
                        pucRequestFrame[ 5 ] = 0x00;
                    }
                    else
                    {
                        return xSetException( ILLEGAL_REQUEST );
                    }

                    break;
                }
#endif
#if( configSFC02 == 1 )
                case RETURN_DIAGNOSTIC_REGISTER:
#endif
#if( configSFC04 == 1 )
                case FORCE_LISTEN_ONLY_MODE:
#endif
#if( configSFC10 == 1 )
                case CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER:
#endif
#if( configSFC11 == 1 )
                case RETURN_BUS_MESSAGE_COUNT:
#endif
#if( configSFC12 == 1 )
                case RETURN_BUS_COMMUNICATION_ERROR_COUNT:
#endif
#if( configSFC13 == 1 )
                case RETURN_BUS_EXCEPTION_ERROR_COUNT:
#endif
#if( configSFC14 == 1 )
                case RETURN_SERVER_MESSAGE_COUNT:
#endif
#if( configSFC15 == 1 )
                case RETURN_SERVER_NO_RESPONSE_COUNT:
#endif
#if( configSFC16 == 1 )
                case RETURN_SERVER_NAK_COUNT:
#endif
#if( configSFC17 == 1 )
                case RETURN_SERVER_BUSY_COUNT:
#endif
#if( configSFC18 == 1 )
                case RETURN_BUS_CHARACTER_OVERRUN_COUNT:
#endif
#if( configSFC20 == 1 )
                case CLEAR_OVERRUN_COUNTER_AND_FLAG:
#endif
#if( ( configSFC02 == 1 ) || ( configSFC04 == 1 ) || ( configSFC10 == 1 ) || \
     ( configSFC11 == 1 ) || ( configSFC12 == 1 ) || ( configSFC13 == 1 ) || \
     ( configSFC14 == 1 ) || ( configSFC15 == 1 ) || ( configSFC16 == 1 ) || \
     ( configSFC17 == 1 ) || ( configSFC18 == 1 ) || ( configSFC20 == 1 ) )
                {
                    pucRequestFrame[ 4 ] = 0x00;
                    pucRequestFrame[ 5 ] = 0x00;

                    break;
                }
#endif
                default:
                {
                    return xSetException( ILLEGAL_SUB_FUNCTION );
                }
            }

            break;
        }
#endif
#if( configFC16 == 1 )
        case WRITE_MULTIPLE_REGISTERS:
        {
            /* Quantity */
            pucRequestFrame[ 4 ] = highByte( request->dataSize );
            pucRequestFrame[ 5 ] =  lowByte( request->dataSize );
            /* Byte-Count */
            pucRequestFrame[ 6 ] = ( uint8_t ) request->dataSize * 2;

            xRequestLength = 7;

            if( request->data != NULL )
            {
                for( size_t i = 0; i < request->dataSize; i++ )
                {
                    pucRequestFrame[ ( i * 2 ) + 7 ] = highByte( request->data[ i ] );
                    pucRequestFrame[ ( i * 2 ) + 8 ] =  lowByte( request->data[ i ] );
                    xRequestLength += 2;
                }
            }
            else
            {
                return xSetException( ILLEGAL_REQUEST );
            }

            break;
        }
#endif
        default:
        {
            return xSetException( ILLEGAL_FUNCTION );
        }
    }

    pxRequest = request;

    return xSetChecksum( pucRequestFrame, &xRequestLength );
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusClient::process( void )
{
    if( bSkipRequestMap == false )
    {
        if( xProcessRequestMap() != OK )
        {
            ( void ) xSetException( ILLEGAL_REQUEST );
            vSetState( PROCESSING_ERROR );
        }
    }
    else
    {
        bSkipRequestMap = false;
    }

    do
    {
        /* Get the current state and select the associated action. */
        switch( xState )
        {
            case CLIENT_IDLE :
            {
                vClearReplyFrame();

                #if( configMODE == configMODE_ASCII )
                {
                    /* We are in ASCII mode, so we convert the frame to the
                    ASCII format (this also updates the pdu length). */
                    ( void ) xRtuToAscii( pucRequestFrame, &xRequestLength );
                }
                #endif

                ( void ) xSendData( pucRequestFrame, xRequestLength );

                /* Check if we have a broadcast or a normal reqest. */
#if( configMODE == configMODE_RTU )
                if( ucREQUEST_ID == configID_BROADCAST )
#endif
#if( configMODE == configMODE_ASCII )
                if( ucAsciiToByte( pucRequestFrame[ 1 ], pucRequestFrame[ 2 ] ) == configID_BROADCAST )
#endif
                {
                    /* Broadcast */

                    vSetState( WAITING_TURNAROUND_DELAY );
                    vStartTurnaroundDelay();
                }
                else
                {
                    /* Normal request */

                    vSetState( WAITING_FOR_REPLY );
                    vStartResponseTimeout();
                }

                break;
            }

            case WAITING_TURNAROUND_DELAY :
            {
                /* Check if the Turnaround Delay has elapsed. */
                if( bTimeoutTurnaroundDelay() == true )
                {
                    /* Just clear the request buffer and go back to idle. */
                    vClearRequestFrame();
                    vSetState( CLIENT_IDLE );
                }

                break;
            }

            case WAITING_FOR_REPLY :
            {
                if( xReplyLength < configFRAME_LEN_MAX )
                {
                    if( bReceiveByte( pucReplyFrame, &xReplyLength ) == true )
                    {
                        #if( configMODE == configMODE_RTU )
                        {
                            vStartInterFrameDelay();
                            vStartInterCharacterTimeout();
                        }
                        #endif

                        #if( configMODE == configMODE_ASCII )
                        {
                            if( pucReplyFrame[ 0 ] != ( uint8_t ) ':' )
                            {
                                vClearReplyFrame();
                            }
                        }
                        #endif

                        break;
                    }
                }
                else
                {
                    ( void ) xSetException( CHARACTER_OVERRUN );
                    vSetState( PROCESSING_ERROR );
                    break;
                }

                if( bTimeoutResponseTimeout() == true )
                {
                    ( void ) xSetException( NO_REPLY );
                    vSetState( PROCESSING_ERROR );
                    break;
                }

                /* Check if the start of a frame has been received. */
                if( xReplyLength >= configFRAME_LEN_MIN )
                {
                    #if( configMODE == configMODE_RTU )
                    {
                        if( bTimeoutInterCharacterTimeout() == true )
                        {
                            if( xCheckChecksum( pucReplyFrame, xReplyLength ) == OK )
                            {
                                if( ucREPLY_ID == ucREQUEST_ID )
                                {
                                    while( bTimeoutInterFrameDelay() != true );
                                    vSetState( PROCESSING_REPLY );
                                }
                            }
                            else
                            {
                                ( void ) xSetException( ILLEGAL_CHECKSUM );
                                vSetState( PROCESSING_ERROR );
                            }
                        }
                    }
                    #endif

                    #if( configMODE == configMODE_ASCII )
                    {
                        /* Check for the end of the ASCII frame which is marked
                        by a carriage-return ('\r') followed by a variable input
                        delimiter (default: line-feed/'\n'). */
                        if( pucReplyFrame[ xReplyLength - 1 ] == ( uint8_t ) cAsciiInputDelimiter )
                        {
                            if( pucReplyFrame[ xReplyLength - 2 ] == ( uint8_t ) '\r' )
                            {
                                /* From this point we handle the request and
                                reply frames in the rtu format. */
                                ( void ) xAsciiToRtu( pucReplyFrame, &xReplyLength );
                                ( void ) xAsciiToRtu( pucRequestFrame, &xRequestLength );

                                if( xCheckChecksum( pucReplyFrame, xReplyLength ) == OK )
                                {
                                    if( ucREPLY_ID == ucREQUEST_ID )
                                    {
                                        vSetState( PROCESSING_REPLY );
                                        break;
                                    }
                                }
                                else
                                {
                                    ( void ) xSetException( ILLEGAL_CHECKSUM );
                                    vSetState( PROCESSING_ERROR );
                                }
                            }
                        }
                    }
                    #endif
                }

                break;
            }

            case PROCESSING_REPLY :
            {
                switch( ucREPLY_FUNCTION_CODE )
                {
#if( ( configFC03 == 1 ) || ( configFC04 == 1 ) )
                    case READ_HOLDING_REGISTERS :
                    case READ_INPUT_REGISTERS :
                    {
                        vHandlerFC03_04();
                        break;
                    }
#endif
#if( configFC05 == 1 )
                    case WRITE_SINGLE_COIL :
                    {
                        vHandlerFC05();
                        break;
                    }
#endif
#if( configFC06 == 1 )
                    case WRITE_SINGLE_REGISTER :
                    {
                        vHandlerFC06();
                        break;
                    }
#endif
#if( configFC08 == 1 )
                    case DIAGNOSTIC :
                    {
                        vHandlerFC08();
                        break;
                    }
#endif
#if( configFC16 == 1 )
                    case WRITE_MULTIPLE_REGISTERS :
                    {
                        vHandlerFC16();
                        break;
                    }
#endif
                    default :
                    {
                        /* The received reply could not be processed, so we
                        check if it was illegal or an error reply. */
                        if( ucREPLY_FUNCTION_CODE == ( ucREQUEST_FUNCTION_CODE | 0x80 ) )
                        {
                            ( void ) xSetException( ( MBException_t ) ucREPLY_ERROR_CODE );
                        }
                        else
                        {
                            ( void ) xSetException( ILLEGAL_FUNCTION );
                        }

                        vSetState( PROCESSING_ERROR );
                    }
                }

                if( xState != PROCESSING_ERROR )
                {
                    vSetState( CLIENT_IDLE );
                }

                break;
            }

            case PROCESSING_ERROR :
            {
                vClearRequestFrame();
                vClearReplyFrame();

                vSetState( CLIENT_IDLE );

                break;
            }

            default :
            {
                ( void ) xSetException( ILLEGAL_STATE );
                vSetState( CLIENT_IDLE );
            }
        }

        #if( configPROCESS_LOOP_HOOK == 1 )
        {
            /* The process loop hook will only be executed when the state
            mashine is not in the idle state. Otherwise the loop hook would be
            execetued with every run through process(). */
            if( ( vProcessLoopHook != NULL ) && ( xState != CLIENT_IDLE ) )
            {
                ( vProcessLoopHook )();
            }
        }
        #endif
    }
    while( xState != CLIENT_IDLE );

    return xException;
}
/*-----------------------------------------------------------*/

void SerialModbusClient::vHandlerFC03_04( void )
{
    size_t xOffset = 0;

    /* Check the reply byte count. */
    if( ucREPLY_BYTE_COUNT == ( ( uint8_t ) usREQUEST_QUANTITY * 2 ) )
    {
        if( pxRequest->data != NULL )
        {
            xOffset = ( size_t ) ( usREQUEST_ADDRESS - pxRequest->address );

            for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
            {
                pxRequest->data[ i + xOffset ] = usReplyWord( i );
            }
        }

        if( pxRequest->callback != NULL )
        {
            ( pxRequest->callback )();
        }
    }
    else
    {
        ( void ) xSetException( ILLEGAL_BYTE_COUNT );
        vSetState( PROCESSING_ERROR );
    }
}
/*-----------------------------------------------------------*/

void SerialModbusClient::vHandlerFC05( void )
{
    /* Check the reply output address. */
    if( usREPLY_ADDRESS == usREQUEST_ADDRESS )
    {
        /* Check the reply output value. */
        if( usREPLY_COIL_VALUE == usREQUEST_COIL_VALUE )
        {
            if( pxRequest->callback != NULL )
            {
                ( pxRequest->callback )();
            }
        }
        else
        {
            ( void ) xSetException( ILLEGAL_COIL_VALUE );
            vSetState( PROCESSING_ERROR );
        }
    }
    else
    {
        ( void ) xSetException( ILLEGAL_OUTPUT_ADDRESS );
        vSetState( PROCESSING_ERROR );
    }
}
/*-----------------------------------------------------------*/

void SerialModbusClient::vHandlerFC06( void )
{
    /* Check the reply output address. */
    if( usREPLY_ADDRESS == usREQUEST_ADDRESS )
    {
        /* Check the reply output value. */
        if( usREPLY_OUTPUT_VALUE == usREQUEST_OUTPUT_VALUE )
        {
            if( pxRequest->callback != NULL )
            {
                ( pxRequest->callback )();
            }
        }
        else
        {
            ( void ) xSetException( ILLEGAL_OUTPUT_VALUE );
            vSetState( PROCESSING_ERROR );
        }
    }
    else
    {
        ( void ) xSetException( ILLEGAL_OUTPUT_ADDRESS );
        vSetState( PROCESSING_ERROR );
    }
}
/*-----------------------------------------------------------*/

void SerialModbusClient::vHandlerFC08( void )
{
    if( usREPLY_SUB_FUNCTION_CODE == usREQUEST_SUB_FUNCTION_CODE )
    {
        switch( usREPLY_SUB_FUNCTION_CODE )
        {
#if( configSFC00 == 1 )
            case RETURN_QUERY_DATA:
            {
                if( xReplyLength == xRequestLength )
                {
                    for( size_t i = 4; i < xReplyLength - xChecksumLength; i++ )
                    {
                        if( pucReplyFrame[ i ] != pucRequestFrame[ i ] )
                        {
                            ( void ) xSetException( ILLEGAL_QUERY_DATA );
                            vSetState( PROCESSING_ERROR );
                            return;
                        }
                    }
                }
                else
                {
                    ( void ) xSetException( ILLEGAL_QUERY_DATA );
                    vSetState( PROCESSING_ERROR );
                    return;
                }

                break;
            }
#endif
#if( configSFC03 == 1 )
            case CHANGE_ASCII_INPUT_DELIMITER:
            {
                if( usREPLY_INPUT_DELIMITER == usREQUEST_INPUT_DELIMITER )
                {
                    cAsciiInputDelimiter = ( char ) ucREPLY_INPUT_DELIMITER_HI;
                }
                else
                {
                    ( void ) xSetException( ILLEGAL_OUTPUT_VALUE );
                    vSetState( PROCESSING_ERROR );
                    return;
                }

                break;
            }
#endif
#if( configSFC01 == 1 )
            case RESTART_COMMUNICATIONS_OPTION:
#endif
#if( configSFC04 == 1 )
            case FORCE_LISTEN_ONLY_MODE:
#endif
#if( configSFC10 == 1 )
            case CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER:
#endif
#if( configSFC20 == 1 )
            case CLEAR_OVERRUN_COUNTER_AND_FLAG:
#endif
#if( ( configSFC01 == 1 ) || ( configSFC04 == 1 ) || \
     ( configSFC10 == 1 ) || ( configSFC20 == 1 ) )
            {
                if( usREPLY_DATA != usREQUEST_DATA )
                {
                    ( void ) xSetException( ILLEGAL_OUTPUT_VALUE );
                    vSetState( PROCESSING_ERROR );
                    return;
                }

                break;
            }
#endif
#if( configSFC02 == 1 )
            case RETURN_DIAGNOSTIC_REGISTER:
#endif
#if( configSFC11 == 1 )
            case RETURN_BUS_MESSAGE_COUNT:
#endif
#if( configSFC12 == 1 )
            case RETURN_BUS_COMMUNICATION_ERROR_COUNT:
#endif
#if( configSFC13 == 1 )
            case RETURN_BUS_EXCEPTION_ERROR_COUNT:
#endif
#if( configSFC14 == 1 )
            case RETURN_SERVER_MESSAGE_COUNT:
#endif
#if( configSFC15 == 1 )
            case RETURN_SERVER_NO_RESPONSE_COUNT:
#endif
#if( configSFC16 == 1 )
            case RETURN_SERVER_NAK_COUNT:
#endif
#if( configSFC17 == 1 )
            case RETURN_SERVER_BUSY_COUNT:
#endif
#if( configSFC18 == 1 )
            case RETURN_BUS_CHARACTER_OVERRUN_COUNT:
#endif
#if( ( configSFC02 == 1 ) || ( configSFC11 == 1 ) || ( configSFC12 == 1 ) || \
     ( configSFC13 == 1 ) || ( configSFC14 == 1 ) || ( configSFC15 == 1 ) || \
     ( configSFC16 == 1 ) || ( configSFC17 == 1 ) || ( configSFC18 == 1 ) )
            {
                if( pxRequest->data != NULL )
                {
                    pxRequest->data[ 0 ] = usREPLY_DATA;
                }

                break;
            }
#endif
            default:
            {
                ( void ) xSetException( ILLEGAL_SUB_FUNCTION );
                vSetState( PROCESSING_ERROR );
                return;
            }
        }

        if( pxRequest->callback != NULL )
        {
            ( pxRequest->callback )();
        }
    }
    else
    {
        ( void ) xSetException( ILLEGAL_REPLY_SUB_FUNCTION );
        vSetState( PROCESSING_ERROR );
    }
}
/*-----------------------------------------------------------*/

void SerialModbusClient::vHandlerFC16( void )
{
    /* Check the reply output address. */
    if( usREPLY_ADDRESS == usREQUEST_ADDRESS )
    {
        /* Check the reply output quantity. */
        if( usREPLY_QUANTITY == usREQUEST_QUANTITY )
        {
            if( pxRequest->callback != NULL )
            {
                ( pxRequest->callback )();
            }
        }
        else
        {
            ( void ) xSetException( ILLEGAL_QUANTITY );
            vSetState( PROCESSING_ERROR );
        }
    }
    else
    {
        ( void ) xSetException( ILLEGAL_OUTPUT_ADDRESS );
        vSetState( PROCESSING_ERROR );
    }
}
/*-----------------------------------------------------------*/

int16_t SerialModbusClient::sendRequest( uint8_t id, uint8_t functionCode, uint16_t address, uint16_t data )
{
    uint16_t usData = 0x0000;
    MBRequest_t xRequest = { 0, 0x00, 0x0000, &usData, 1, NULL };

    xRequest.id = id;
    xRequest.functionCode = functionCode;
    xRequest.address = address;
    usData = data;

    xStatusSimpleAPI = setRequest( &xRequest );
    if( xStatusSimpleAPI == OK )
    {
        xStatusSimpleAPI = process();
        if( xStatusSimpleAPI == OK )
        {
            return ( int16_t ) usData;
        }
    }

    return -1;
}
/*-----------------------------------------------------------*/

int16_t SerialModbusClient::readHoldingRegister( uint8_t id, uint16_t address )
{
    return sendRequest( id, READ_HOLDING_REGISTERS, address, 0x0000 );
}
/*-----------------------------------------------------------*/

int16_t SerialModbusClient::readInputRegister( uint8_t id, uint16_t address )
{
    return sendRequest( id, READ_INPUT_REGISTERS, address, 0x0000 );
}
/*-----------------------------------------------------------*/

int16_t SerialModbusClient::writeSingleCoil( uint8_t id, uint16_t address, uint16_t data )
{
    return sendRequest( id, WRITE_SINGLE_COIL, address, data );
}
/*-----------------------------------------------------------*/

int16_t SerialModbusClient::writeSingleRegister( uint8_t id, uint16_t address, uint16_t data )
{
    return sendRequest( id, WRITE_SINGLE_REGISTER, address, data );
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusClient::getLastException( void )
{
    return xStatusSimpleAPI;
}
/*-----------------------------------------------------------*/

const char * SerialModbusClient::getLastExceptionString( void )
{
    return getExceptionString( xStatusSimpleAPI );
}
