////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SerialModbusClient.cpp
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

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include "SerialModbusConfig.h"
#include "SerialModbusBase.h"
#include "SerialModbusClient.h"

#include <Arduino.h>

/*----------------------------------------------------------------------------*/

SerialModbusClient::SerialModbusClient()
{
    xState = CLIENT_IDLE;

    pxRequestMap = NULL;
    xRequestMapIndex = 0;
    pxRequest = NULL;
    bSkipRequestMap = false;

    ulTurnaroundDelayMs = configMB_TURNAROUND_DELAY_MS;
    ulResponseTimeoutMs = configMB_RESPONSE_TIMEOUT_MS;

    ulTimerResponseTimeoutMs = 0;
    ulTimerTurnaroundDelayMs = 0;

    xStatusSimpleAPI = MB_OK;
}
/*----------------------------------------------------------------------------*/

void SerialModbusClient::vSetState( MB_ClientState_t xStatePar )
{
    xState = xStatePar;
}
/*----------------------------------------------------------------------------*/

void SerialModbusClient::vStartTurnaroundDelay( void )
{
    ulTimerTurnaroundDelayMs = millis();
}
/*----------------------------------------------------------------------------*/

void SerialModbusClient::vStartResponseTimeout( void )
{
    ulTimerResponseTimeoutMs = millis();
}
/*----------------------------------------------------------------------------*/

bool SerialModbusClient::bTimeoutTurnaroundDelay( void )
{
    return ( millis() - ulTimerTurnaroundDelayMs ) >= ulTurnaroundDelayMs;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusClient::bTimeoutResponseTimeout( void )
{
    return ( millis() - ulTimerResponseTimeoutMs ) >= ulResponseTimeoutMs;
}
/*----------------------------------------------------------------------------*/

uint32_t SerialModbusClient::getResponseTimeout( void )
{
    return ulResponseTimeoutMs;
}
/*----------------------------------------------------------------------------*/

uint32_t SerialModbusClient::getTurnaroundDelay( void )
{
    return ulTurnaroundDelayMs;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusClient::setResponseTimeout( uint32_t timeMs )
{
    if( timeMs != 0 )
    {
        ulResponseTimeoutMs = timeMs;
        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusClient::setTurnaroundDelay( uint32_t timeMs )
{
    if( timeMs != 0 )
    {
        ulTurnaroundDelayMs = timeMs;
        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

void SerialModbusClient::setRequestMap( const MB_Request_t * requestMap )
{
    pxRequestMap = requestMap;
    xRequestMapIndex = 0;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusClient::xProcessRequestMap( void )
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

    return MB_OK;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusClient::setRequest( const MB_Request_t * request, bool requestMap )
{
    if( request == NULL )
    {
        return xSetException( MB_ILLEGAL_REQUEST );
    }

    if( requestMap == false )
    {
        bSkipRequestMap = true;
    }

    ( void ) xSetException( MB_OK );

    if( ( request->id > configMB_ID_SERVER_MAX ) ||
        ( request->functionCode == 0x00 ) ||
        ( request->dataSize == 0 ) )
    {
        return xSetException( MB_ILLEGAL_REQUEST );
    }

    vClearRequestFrame();

    pucRequestFrame[ 0 ] = request->id;
    pucRequestFrame[ 1 ] = request->functionCode;
    pucRequestFrame[ 2 ] = highByte( request->address );
    pucRequestFrame[ 3 ] =  lowByte( request->address );

    switch( request->functionCode )
    {
#if( ( configMB_FC03 == 1 ) || ( configMB_FC04 == 1 ) )
        case FC_READ_HOLDING_REGISTERS:
        case FC_READ_INPUT_REGISTERS:
        {
            pucRequestFrame[ 4 ] = highByte( request->dataSize );
            pucRequestFrame[ 5 ] =  lowByte( request->dataSize );
            xRequestLength = 6;

            break;
        }
#endif
#if( ( configMB_FC05 == 1 ) || ( configMB_FC06 == 1 ) )
        case FC_WRITE_SINGLE_COIL:
        case FC_WRITE_SINGLE_REGISTER:
        {
            if( request->data != NULL )
            {
                pucRequestFrame[ 4 ] = highByte( request->data[ 0 ] );
                pucRequestFrame[ 5 ] =  lowByte( request->data[ 0 ] );
                xRequestLength = 6;
            }
            else
            {
                return xSetException( MB_ILLEGAL_REQUEST );
            }

            break;
        }
#endif
#if( configMB_FC08 == 1 )
        case FC_DIAGNOSTIC:
        {
            xRequestLength = 6;

            switch( request->address )
            {
#if( configMB_SFC00 == 1 )
                case SFC_RETURN_QUERY_DATA:
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
                        return xSetException( MB_ILLEGAL_REQUEST );
                    }

                    break;
                }
#endif
#if( configMB_SFC01 == 1 )
                case SFC_RESTART_COMMUNICATIONS_OPTION:
                {
                    if( request->data != NULL )
                    {
                        pucRequestFrame[ 4 ] = highByte( request->data[ 0 ] );
                        pucRequestFrame[ 5 ] =  lowByte( request->data[ 0 ] );
                    }
                    else
                    {
                        return xSetException( MB_ILLEGAL_REQUEST );
                    }

                    break;
                }
#endif
#if( configMB_SFC03 == 1 )
                case SFC_CHANGE_ASCII_INPUT_DELIMITER:
                {
                    if( isAscii( *( ( char * ) request->data ) ) == true )
                    {
                        pucRequestFrame[ 4 ] = ( uint8_t ) *( ( char * ) request->data );
                        pucRequestFrame[ 5 ] = 0x00;
                    }
                    else
                    {
                        return xSetException( MB_ILLEGAL_REQUEST );
                    }

                    break;
                }
#endif
#if( configMB_SFC02 == 1 )
                case SFC_RETURN_DIAGNOSTIC_REGISTER:
#endif
#if( configMB_SFC04 == 1 )
                case SFC_FORCE_LISTEN_ONLY_MODE:
#endif
#if( configMB_SFC10 == 1 )
                case SFC_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER:
#endif
#if( configMB_SFC11 == 1 )
                case SFC_RETURN_BUS_MESSAGE_COUNT:
#endif
#if( configMB_SFC12 == 1 )
                case SFC_RETURN_BUS_COMMUNICATION_ERROR_COUNT:
#endif
#if( configMB_SFC13 == 1 )
                case SFC_RETURN_BUS_EXCEPTION_ERROR_COUNT:
#endif
#if( configMB_SFC14 == 1 )
                case SFC_RETURN_SERVER_MESSAGE_COUNT:
#endif
#if( configMB_SFC15 == 1 )
                case SFC_RETURN_SERVER_NO_RESPONSE_COUNT:
#endif
#if( configMB_SFC16 == 1 )
                case SFC_RETURN_SERVER_NAK_COUNT:
#endif
#if( configMB_SFC17 == 1 )
                case SFC_RETURN_SERVER_BUSY_COUNT:
#endif
#if( configMB_SFC18 == 1 )
                case SFC_RETURN_BUS_CHARACTER_OVERRUN_COUNT:
#endif
#if( configMB_SFC20 == 1 )
                case SFC_CLEAR_OVERRUN_COUNTER_AND_FLAG:
#endif
#if( ( configMB_SFC02 == 1 ) || ( configMB_SFC04 == 1 ) || ( configMB_SFC10 == 1 ) || \
     ( configMB_SFC11 == 1 ) || ( configMB_SFC12 == 1 ) || ( configMB_SFC13 == 1 ) || \
     ( configMB_SFC14 == 1 ) || ( configMB_SFC15 == 1 ) || ( configMB_SFC16 == 1 ) || \
     ( configMB_SFC17 == 1 ) || ( configMB_SFC18 == 1 ) || ( configMB_SFC20 == 1 ) )
                {
                    pucRequestFrame[ 4 ] = 0x00;
                    pucRequestFrame[ 5 ] = 0x00;

                    break;
                }
#endif
                default:
                {
                    return xSetException( MB_ILLEGAL_SUB_FUNCTION );
                }
            }

            break;
        }
#endif
#if( configMB_FC16 == 1 )
        case FC_WRITE_MULTIPLE_REGISTERS:
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
                return xSetException( MB_ILLEGAL_REQUEST );
            }

            break;
        }
#endif
        default:
        {
            return xSetException( MB_ILLEGAL_FUNCTION );
        }
    }

    pxRequest = request;

    return xSetChecksum( pucRequestFrame, &xRequestLength );
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusClient::process( void )
{
    if( bSkipRequestMap == false )
    {
        if( xProcessRequestMap() != MB_OK )
        {
            ( void ) xSetException( MB_ILLEGAL_REQUEST );
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
            case CLIENT_IDLE : /*---------------------------------------------*/
            {
                vClearReplyFrame();

                #if( configMB_MODE == configMB_MODE_ASCII )
                {
                    /* We are in ASCII mode, so we convert the frame to the
                     * ASCII format (this also updates the pdu length). */
                    ( void ) xRtuToAscii( pucRequestFrame, &xRequestLength );
                }
                #endif

                ( void ) xSendData( pucRequestFrame, xRequestLength );

                /* Check if we have a broadcast or a normal reqest. */
#if( configMB_MODE == configMB_MODE_RTU )
                if( ucREQUEST_ID == configMB_ID_BROADCAST )
#endif
#if( configMB_MODE == configMB_MODE_ASCII )
                if( ucAsciiToByte( pucRequestFrame[ 1 ], pucRequestFrame[ 2 ] ) == configMB_ID_BROADCAST )
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

            case WAITING_TURNAROUND_DELAY : /*--------------------------------*/
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

            case WAITING_FOR_REPLY : /*---------------------------------------*/
            {
                if( xReplyLength < configMB_FRAME_LEN_MAX )
                {
                    if( bReceiveByte( pucReplyFrame, &xReplyLength ) == true )
                    {
                        #if( configMB_MODE == configMB_MODE_RTU )
                        {
                            vStartInterFrameDelay();
                            vStartInterCharacterTimeout();
                        }
                        #endif

                        #if( configMB_MODE == configMB_MODE_ASCII )
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
                    ( void ) xSetException( MB_CHARACTER_OVERRUN );
                    vSetState( PROCESSING_ERROR );
                    break;
                }

                if( bTimeoutResponseTimeout() == true )
                {
                    ( void ) xSetException( MB_NO_REPLY );
                    vSetState( PROCESSING_ERROR );
                    break;
                }

                /* Check if the start of a frame has been received. */
                if( xReplyLength >= configMB_FRAME_LEN_MIN )
                {
                    #if( configMB_MODE == configMB_MODE_RTU )
                    {
                        if( bTimeoutInterCharacterTimeout() == true )
                        {
                            if( xCheckChecksum( pucReplyFrame, xReplyLength ) == MB_OK )
                            {
                                if( ucREPLY_ID == ucREQUEST_ID )
                                {
                                    while( bTimeoutInterFrameDelay() != true );
                                    vSetState( PROCESSING_REPLY );
                                }
                            }
                            else
                            {
                                ( void ) xSetException( MB_ILLEGAL_CHECKSUM );
                                vSetState( PROCESSING_ERROR );
                            }
                        }
                    }
                    #endif

                    #if( configMB_MODE == configMB_MODE_ASCII )
                    {
                        /* Check for the end of the ASCII frame which is marked
                         * by a carriage-return ('\r') followed by a variable
                         * input delimiter (default: line-feed/'\n'). */
                        if( pucReplyFrame[ xReplyLength - 1 ] == ( uint8_t ) cAsciiInputDelimiter )
                        {
                            if( pucReplyFrame[ xReplyLength - 2 ] == ( uint8_t ) '\r' )
                            {
                                /* From this point we handle the request and
                                 * reply frames in the rtu format. */
                                ( void ) xAsciiToRtu( pucReplyFrame, &xReplyLength );
                                ( void ) xAsciiToRtu( pucRequestFrame, &xRequestLength );

                                if( xCheckChecksum( pucReplyFrame, xReplyLength ) == MB_OK )
                                {
                                    if( ucREPLY_ID == ucREQUEST_ID )
                                    {
                                        vSetState( PROCESSING_REPLY );
                                        break;
                                    }
                                }
                                else
                                {
                                    ( void ) xSetException( MB_ILLEGAL_CHECKSUM );
                                    vSetState( PROCESSING_ERROR );
                                }
                            }
                        }
                    }
                    #endif
                }

                break;
            }

            case PROCESSING_REPLY : /*----------------------------------------*/
            {
                switch( ucREPLY_FUNCTION_CODE )
                {
#if( ( configMB_FC03 == 1 ) || ( configMB_FC04 == 1 ) )
                    case FC_READ_HOLDING_REGISTERS :
                    case FC_READ_INPUT_REGISTERS :
                    {
                        vHandlerFC03_04();
                        break;
                    }
#endif
#if( configMB_FC05 == 1 )
                    case FC_WRITE_SINGLE_COIL :
                    {
                        vHandlerFC05();
                        break;
                    }
#endif
#if( configMB_FC06 == 1 )
                    case FC_WRITE_SINGLE_REGISTER :
                    {
                        vHandlerFC06();
                        break;
                    }
#endif
#if( configMB_FC08 == 1 )
                    case FC_DIAGNOSTIC :
                    {
                        vHandlerFC08();
                        break;
                    }
#endif
#if( configMB_FC16 == 1 )
                    case FC_WRITE_MULTIPLE_REGISTERS :
                    {
                        vHandlerFC16();
                        break;
                    }
#endif
                    default :
                    {
                        /* The received reply could not be processed, so we
                         * check if it was illegal or an error reply. */
                        if( ucREPLY_FUNCTION_CODE == ( ucREQUEST_FUNCTION_CODE | 0x80 ) )
                        {
                            ( void ) xSetException( ( MB_Exception_t ) ucREPLY_ERROR_CODE );
                        }
                        else
                        {
                            ( void ) xSetException( MB_ILLEGAL_FUNCTION );
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

            case PROCESSING_ERROR : /*----------------------------------------*/
            {
                vClearRequestFrame();
                vClearReplyFrame();

                vSetState( CLIENT_IDLE );

                break;
            }

            default :
            {
                ( void ) xSetException( MB_ILLEGAL_STATE );
                vSetState( CLIENT_IDLE );
            }
        }

        #if( configMB_PROCESS_LOOP_HOOK == 1 )
        {
            /* The process loop hook will only be executed when the state
             * mashine is not in the idle state. Otherwise the loop hook would
             * be execetued with every run through process(). */
            if( ( vProcessLoopHook != NULL ) && ( xState != CLIENT_IDLE ) )
            {
                ( vProcessLoopHook )();
            }
        }
        #endif
    }
    while( xState != CLIENT_IDLE );

    return xStatus;
}
/*----------------------------------------------------------------------------*/

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
        ( void ) xSetException( MB_ILLEGAL_BYTE_COUNT );
        vSetState( PROCESSING_ERROR );
    }
}
/*----------------------------------------------------------------------------*/

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
            ( void ) xSetException( MB_ILLEGAL_COIL_VALUE );
            vSetState( PROCESSING_ERROR );
        }
    }
    else
    {
        ( void ) xSetException( MB_ILLEGAL_OUTPUT_ADDRESS );
        vSetState( PROCESSING_ERROR );
    }
}
/*----------------------------------------------------------------------------*/

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
            ( void ) xSetException( MB_ILLEGAL_OUTPUT_VALUE );
            vSetState( PROCESSING_ERROR );
        }
    }
    else
    {
        ( void ) xSetException( MB_ILLEGAL_OUTPUT_ADDRESS );
        vSetState( PROCESSING_ERROR );
    }
}
/*----------------------------------------------------------------------------*/

void SerialModbusClient::vHandlerFC08( void )
{
    if( usREPLY_SUB_FUNCTION_CODE == usREQUEST_SUB_FUNCTION_CODE )
    {
        switch( usREPLY_SUB_FUNCTION_CODE )
        {
#if( configMB_SFC00 == 1 )
            case SFC_RETURN_QUERY_DATA:
            {
                if( xReplyLength == xRequestLength )
                {
                    for( size_t i = 4; i < xReplyLength - xChecksumLength; i++ )
                    {
                        if( pucReplyFrame[ i ] != pucRequestFrame[ i ] )
                        {
                            ( void ) xSetException( MB_ILLEGAL_QUERY_DATA );
                            vSetState( PROCESSING_ERROR );
                            return;
                        }
                    }
                }
                else
                {
                    ( void ) xSetException( MB_ILLEGAL_QUERY_DATA );
                    vSetState( PROCESSING_ERROR );
                    return;
                }

                break;
            }
#endif
#if( configMB_SFC03 == 1 )
            case SFC_CHANGE_ASCII_INPUT_DELIMITER:
            {
                if( usREPLY_INPUT_DELIMITER == usREQUEST_INPUT_DELIMITER )
                {
                    cAsciiInputDelimiter = ( char ) ucREPLY_INPUT_DELIMITER_HI;
                }
                else
                {
                    ( void ) xSetException( MB_ILLEGAL_OUTPUT_VALUE );
                    vSetState( PROCESSING_ERROR );
                    return;
                }

                break;
            }
#endif
#if( configMB_SFC01 == 1 )
            case SFC_RESTART_COMMUNICATIONS_OPTION:
#endif
#if( configMB_SFC04 == 1 )
            case SFC_FORCE_LISTEN_ONLY_MODE:
#endif
#if( configMB_SFC10 == 1 )
            case SFC_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER:
#endif
#if( configMB_SFC20 == 1 )
            case SFC_CLEAR_OVERRUN_COUNTER_AND_FLAG:
#endif
#if( ( configMB_SFC01 == 1 ) || ( configMB_SFC04 == 1 ) || \
     ( configMB_SFC10 == 1 ) || ( configMB_SFC20 == 1 ) )
            {
                if( usREPLY_DATA != usREQUEST_DATA )
                {
                    ( void ) xSetException( MB_ILLEGAL_OUTPUT_VALUE );
                    vSetState( PROCESSING_ERROR );
                    return;
                }

                break;
            }
#endif
#if( configMB_SFC02 == 1 )
            case SFC_RETURN_DIAGNOSTIC_REGISTER:
#endif
#if( configMB_SFC11 == 1 )
            case SFC_RETURN_BUS_MESSAGE_COUNT:
#endif
#if( configMB_SFC12 == 1 )
            case SFC_RETURN_BUS_COMMUNICATION_ERROR_COUNT:
#endif
#if( configMB_SFC13 == 1 )
            case SFC_RETURN_BUS_EXCEPTION_ERROR_COUNT:
#endif
#if( configMB_SFC14 == 1 )
            case SFC_RETURN_SERVER_MESSAGE_COUNT:
#endif
#if( configMB_SFC15 == 1 )
            case SFC_RETURN_SERVER_NO_RESPONSE_COUNT:
#endif
#if( configMB_SFC16 == 1 )
            case SFC_RETURN_SERVER_NAK_COUNT:
#endif
#if( configMB_SFC17 == 1 )
            case SFC_RETURN_SERVER_BUSY_COUNT:
#endif
#if( configMB_SFC18 == 1 )
            case SFC_RETURN_BUS_CHARACTER_OVERRUN_COUNT:
#endif
#if( ( configMB_SFC02 == 1 ) || ( configMB_SFC11 == 1 ) || ( configMB_SFC12 == 1 ) || \
     ( configMB_SFC13 == 1 ) || ( configMB_SFC14 == 1 ) || ( configMB_SFC15 == 1 ) || \
     ( configMB_SFC16 == 1 ) || ( configMB_SFC17 == 1 ) || ( configMB_SFC18 == 1 ) )
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
                ( void ) xSetException( MB_ILLEGAL_SUB_FUNCTION );
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
        ( void ) xSetException( MB_ILLEGAL_REPLY_SUB_FUNCTION );
        vSetState( PROCESSING_ERROR );
    }
}
/*----------------------------------------------------------------------------*/

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
            ( void ) xSetException( MB_ILLEGAL_QUANTITY );
            vSetState( PROCESSING_ERROR );
        }
    }
    else
    {
        ( void ) xSetException( MB_ILLEGAL_OUTPUT_ADDRESS );
        vSetState( PROCESSING_ERROR );
    }
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusClient::sendRequest( uint8_t id, uint8_t functionCode, uint16_t address, uint16_t data )
{
    MB_Request_t xRequest = { 0xFF, 0x00, 0xFFFF, &data, 1, NULL };

    xRequest.id = id;
    xRequest.functionCode = functionCode;
    xRequest.address = address;

    xStatusSimpleAPI = setRequest( &xRequest );
    if( xStatusSimpleAPI == MB_OK )
    {
        xStatusSimpleAPI = process();
        if( xStatusSimpleAPI == MB_OK )
        {
            return ( int32_t ) data;
        }
    }

    return -1;
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusClient::readHoldingRegister( uint8_t id, uint16_t address )
{
    return sendRequest( id, FC_READ_HOLDING_REGISTERS, address, 0x0000 );
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusClient::readInputRegister( uint8_t id, uint16_t address )
{
    return sendRequest( id, FC_READ_INPUT_REGISTERS, address, 0x0000 );
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusClient::writeSingleCoil( uint8_t id, uint16_t address, uint16_t data )
{
    return sendRequest( id, FC_WRITE_SINGLE_COIL, address, data );
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusClient::writeSingleRegister( uint8_t id, uint16_t address, uint16_t data )
{
    return sendRequest( id, FC_WRITE_SINGLE_REGISTER, address, data );
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusClient::getLastException( void )
{
    return xStatusSimpleAPI;
}
/*----------------------------------------------------------------------------*/

const char * SerialModbusClient::getLastExceptionString( void )
{
    return getExceptionString( xStatusSimpleAPI );
}
