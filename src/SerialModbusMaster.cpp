////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusMaster.cpp
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
#include <ctype.h>

#include "SerialModbusConfig.h"
#include "SerialModbusBase.h"
#include "SerialModbusMaster.h"

#include <Arduino.h>

/*-----------------------------------------------------------*/

SerialModbusMaster::SerialModbusMaster()
{
    xState = MASTER_IDLE;

    xReplyDataSize = 0;

    pxRequest = NULL;

    ulTurnaroundDelayUs = configTURNAROUND_DELAY_US;
    ulResponseTimeoutUs = configRESPONSE_TIMEOUT_US;

    ulTimerResponseTimeoutUs = 0;
    ulTimerTurnaroundDelayUs = 0;

    pxRequestMap = NULL;
}
/*-----------------------------------------------------------*/

void SerialModbusMaster::vSetState( MBMasterState_t xStatePar )
{
    xState = xStatePar;
}
/*-----------------------------------------------------------*/

void SerialModbusMaster::vStartTurnaroundDelay( void )
{
    ulTimerTurnaroundDelayUs = micros();
}
/*-----------------------------------------------------------*/

void SerialModbusMaster::vStartResponseTimeout( void )
{
    ulTimerResponseTimeoutUs = micros();
}
/*-----------------------------------------------------------*/

bool SerialModbusMaster::bTimeoutTurnaroundDelay( void ) const
{
    return ( micros() - ulTimerTurnaroundDelayUs ) >= ulTurnaroundDelayUs;
}
/*-----------------------------------------------------------*/

bool SerialModbusMaster::bTimeoutResponseTimeout( void ) const
{
    return ( micros() - ulTimerResponseTimeoutUs ) >= ulResponseTimeoutUs;
}
/*-----------------------------------------------------------*/

void SerialModbusMaster::setResponseTimeout( uint32_t timeMs )
{
    ulResponseTimeoutUs = timeMs * 1000;
}
/*-----------------------------------------------------------*/

void SerialModbusMaster::setTurnaroundDelay( uint32_t timeMs )
{
    ulTurnaroundDelayUs = timeMs * 1000;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusMaster::xProcessRequestMap( void )
{
    static size_t xRequestMapIndex = 0;

    if( pxRequestMap != NULL )
    {
        if( pxRequestMap[ xRequestMapIndex ].functionCode == 0x00 )
        {
            xRequestMapIndex = 0;
        }

        return setRequest( &pxRequestMap[ xRequestMapIndex++ ] );
    }
    else
    {
        xRequestMapIndex = 0;
    }

    return OK;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusMaster::setRequest( const MBRequest_t * request )
{
    if( request == NULL )
    {
        return xSetException( ILLEGAL_REQUEST );
    }

    xSetException( OK );

    if( ( request->id           >  configID_SLAVE_MAX ) ||
        ( request->functionCode == 0x00               ) ||
        ( request->objectSize   == 0                  ) )
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
            pucRequestFrame[ 4 ] = highByte( request->objectSize );
            pucRequestFrame[ 5 ] =  lowByte( request->objectSize );
            xRequestLength = 6;

            break;
        }
#endif
#if( ( configFC05 == 1 ) || ( configFC06 == 1 ) )
        case WRITE_SINGLE_COIL:
        case WRITE_SINGLE_REGISTER:
        {
            if( request->object != NULL )
            {
                pucRequestFrame[ 4 ] = highByte( ( ( uint16_t * ) request->object )[ 0 ] );
                pucRequestFrame[ 5 ] =  lowByte( ( ( uint16_t * ) request->object )[ 0 ] );
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
                    xRequestLength = 4;

                    if( request->object != NULL )
                    {
                        for( size_t i = xRequestLength; i < request->objectSize; i++ )
                        {
                            pucRequestFrame[ i ] = ( ( uint8_t * ) request->object )[ i ];
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
                    // TODO
                    break;
                }
#endif
#if( configSFC03 == 1 )
                case CHANGE_ASCII_INPUT_DELIMITER:
                {
                    if( isAscii( ( int ) request->object ) == true )
                    {
                        pucRequestFrame[ 4 ] = highByte( ( ( uint16_t * ) request->object )[ 0 ] );
                        pucRequestFrame[ 5 ] =  lowByte( ( ( uint16_t * ) request->object )[ 0 ] );
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
                case RETURN_SLAVE_MESSAGE_COUNT:
#endif
#if( configSFC15 == 1 )
                case RETURN_SLAVE_NO_RESPONSE_COUNT:
#endif
#if( configSFC16 == 1 )
                case RETURN_SLAVE_NAK_COUNT:
#endif
#if( configSFC17 == 1 )
                case RETURN_SLAVE_BUSY_COUNT:
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
                    xRequestLength = 6;

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
            pucRequestFrame[ 4 ] = highByte( request->objectSize );
            pucRequestFrame[ 5 ] =  lowByte( request->objectSize );
            /* Byte-Count */
            pucRequestFrame[ 6 ] = ( uint8_t ) request->objectSize * 2;

            xRequestLength = 7;

            if( request->object != NULL )
            {
                for( size_t i = 0; i < request->objectSize; i++ )
                {
                    pucRequestFrame[ ( i * 2 ) + 7 ] = highByte( ( ( uint16_t * ) request->object )[ i ] );
                    pucRequestFrame[ ( i * 2 ) + 8 ] =  lowByte( ( ( uint16_t * ) request->object )[ i ] );
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

void SerialModbusMaster::setRequestMap( const MBRequest_t * requestMap )
{
    pxRequestMap = requestMap;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusMaster::processModbus( void )
{
    if( xProcessRequestMap() != OK )
    {
        xSetException( ILLEGAL_REQUEST );
        vSetState( PROCESSING_ERROR );
    }

    do
    {
        /* Get the current state and select the associated action. */
        switch( xState )
        {
            case MASTER_IDLE :
            {
                vClearReplyFrame();

                #if( configMODE == configMODE_ASCII )
                {
                    /* Convert request to ascii and update the pdu length */
                    xRtuToAscii( pucRequestFrame, &xRequestLength );
                }
                #endif

                vSendData( pucRequestFrame, xRequestLength );

                /* Check for broadcast or normal reqest */
#if( configMODE == configMODE_RTU )
                if( ucREQUEST_ID == configID_BROADCAST )
#endif
#if( configMODE == configMODE_ASCII )
                if( ucAsciiToByte( pucRequestFrame[ 1 ], pucRequestFrame[ 2 ] ) == configID_BROADCAST )
#endif
                {
                    /* Broadcast (request id is 0) */
                    vSetState( WAITING_TURNAROUND_DELAY );

                    /* Start timer for turnaround-delay */
                    vStartTurnaroundDelay();
                }
                else
                {
                    /* Normal request */
                    vSetState( WAITING_FOR_REPLY );

                    /* Start timer for resposne-timeout */
                    vStartResponseTimeout();
                }

                break;
            }

            case WAITING_TURNAROUND_DELAY :
            {
                if( bTimeoutTurnaroundDelay() == true )
                {
                    /* Nothing ... clear buffer and go on */
                    vClearRequestFrame();
                    vSetState( MASTER_IDLE );
                }

                break;
            }

            case WAITING_FOR_REPLY :
            {
                if( xReplyLength < configMAX_FRAME_SIZE )
                {
                    if( bReceiveByte( pucReplyFrame, &xReplyLength ) == true )
                    {
                        #if( configMODE == configMODE_RTU )
                        {
                            if( ucREPLY_ID == ucREQUEST_ID )
                            {
                                vStartInterFrameDelay();
                                vStartInterCharacterTimeout();
                                break;
                            }
                        }
                        #endif

                        #if( configMODE == configMODE_ASCII )
                        {
                            if( pucReplyFrame[ 0 ] == ( uint8_t ) ':' )
                            {
                                break;
                            }
                        }
                        #endif

                        vClearReplyFrame();
                        break;
                    }
                }
                else
                {
                    xSetException( CHARACTER_OVERRUN );
                    vSetState( PROCESSING_ERROR );
                    break;
                }

                if( bTimeoutResponseTimeout() == true )
                {
                    xSetException( NO_REPLY );
                    vSetState( PROCESSING_ERROR );
                    break;
                }

                /* Check if the start of a frame has been received. */
                if( xReplyLength > 0 )
                {
                    #if( configMODE == configMODE_RTU )
                    {
                        if( bTimeoutInterCharacterTimeout() == true )
                        {
                            if( xCheckChecksum( pucReplyFrame, xReplyLength ) == OK )
                            {
                                while( bTimeoutInterFrameDelay() != true );
                                vSetState( PROCESSING_REPLY );
                            }
                            else
                            {
                                xSetException( ILLEGAL_CHECKSUM );
                                vSetState( PROCESSING_ERROR );
                            }
                        }
                    }
                    #endif

                    #if( configMODE == configMODE_ASCII )
                    {
                        /* Check for Newline (frame end) */
                        if( pucReplyFrame[ xReplyLength - 1 ] == ( uint8_t ) cAsciiInputDelimiter )
                        {
                            /* Check for Carriage Return (frame end) */
                            if( pucReplyFrame[ xReplyLength - 2 ] == ( uint8_t ) '\r' )
                            {
                                /* Convert the frame from rtu to ascii format */
                                xAsciiToRtu( pucReplyFrame, &xReplyLength );
                                xAsciiToRtu( pucRequestFrame, &xRequestLength );

                                if( ucREPLY_ID == ucREQUEST_ID )
                                {
                                    if( xCheckChecksum( pucReplyFrame, xReplyLength ) == OK )
                                    {
                                        vSetState( PROCESSING_REPLY );
                                        break;
                                    }
                                }

                                vClearReplyFrame();
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
                        /* Check for Error Code */
                        if( ucREPLY_FUNCTION_CODE == ( ucREQUEST_FUNCTION_CODE | 0x80 ) )
                        {
                            xSetException( ( MBException_t ) ucREPLY_ERROR_CODE );
                        }
                        else
                        {
                            xSetException( ILLEGAL_FUNCTION );
                        }

                        vSetState( PROCESSING_ERROR );
                    }
                }

                if( xState != PROCESSING_ERROR )
                {
                    vSetState( MASTER_IDLE );
                }

                break;
            }

            case PROCESSING_ERROR :
            {
                vClearRequestFrame();
                vClearReplyFrame();
                xReplyDataSize = 0;

                vSetState( MASTER_IDLE );

                break;
            }

            default :
            {
                xSetException( ILLEGAL_STATE );
                vSetState( MASTER_IDLE );
            }
        }

        #if( configPROCESS_LOOP_HOOK == 1 )
        {
            /* The process loop hook will only be executed when the state
            mashine is not in the idle state. Otherwise the loop hook would be
            execetued with every run through processModbus(). */
            if( ( vProcessLoopHook != NULL ) && ( xState != MASTER_IDLE ) )
            {
                (*vProcessLoopHook)();
            }
        }
        #endif
    }
    while( xState != MASTER_IDLE );

    return xException;
}
/*-----------------------------------------------------------*/

#if( ( configFC03 == 1 ) || ( configFC04 == 1 ) )

    void SerialModbusMaster::vHandlerFC03_04( void )
    {
        size_t xOffset = 0;

        /* Check the response byte count */
        if( ucREPLY_BYTE_COUNT == ( uint8_t ) ( 2 * usREQUEST_QUANTITY ) )
        {
            if( pxRequest->object != NULL )
            {
                xOffset = ( size_t ) ( usREQUEST_ADDRESS - pxRequest->address );
                
                for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
                {
                    ( ( uint16_t * ) pxRequest->object )[ i + xOffset ] = usReplyWord( i );
                }
            }

            xReplyDataSize = ( size_t ) usREQUEST_QUANTITY;

            if( pxRequest->action != NULL )
            {
                (*pxRequest->action)();
            }
        }
        else
        {
            xSetException( ILLEGAL_BYTE_COUNT );
            vSetState( PROCESSING_ERROR );
        }
    }

#endif
/*-----------------------------------------------------------*/

#if( configFC05 == 1 )

    void SerialModbusMaster::vHandlerFC05( void )
    {
        /* Check the response output address */
        if( usREPLY_ADDRESS == usREQUEST_ADDRESS )
        {
            /* Check the response output value */
            if( usREPLY_COIL_VALUE == usREQUEST_COIL_VALUE )
            {
                xReplyDataSize = 1;

                if( pxRequest->action != NULL )
                {
                    (*pxRequest->action)();
                }
            }
            else
            {
                xSetException( ILLEGAL_COIL_VALUE );
                vSetState( PROCESSING_ERROR );
            }
        }
        else
        {
            xSetException( ILLEGAL_OUTPUT_ADDRESS );
            vSetState( PROCESSING_ERROR );
        }
    }

#endif
/*-----------------------------------------------------------*/

#if( configFC06 == 1 )

    void SerialModbusMaster::vHandlerFC06( void )
    {
        /* Check the response output address */
        if( usREPLY_ADDRESS == usREQUEST_ADDRESS )
        {
            /* Check the response output value */
            if( usREPLY_OUTPUT_VALUE == usREQUEST_OUTPUT_VALUE )
            {
                xReplyDataSize = 1;

                if( pxRequest->action != NULL )
                {
                    (*pxRequest->action)();
                }
            }
            else
            {
                xSetException( ILLEGAL_OUTPUT_VALUE );
                vSetState( PROCESSING_ERROR );
            }
        }
        else
        {
            xSetException( ILLEGAL_OUTPUT_ADDRESS );
            vSetState( PROCESSING_ERROR );
        }
    }

#endif
/*-----------------------------------------------------------*/

#if( configFC08 == 1 )

    void SerialModbusMaster::vHandlerFC08( void )
    {
        if( usREPLY_SUB_FUNCTION_CODE == usREQUEST_SUB_FUNCTION_CODE )
        {
            switch( usREPLY_SUB_FUNCTION_CODE )
            {
#if( configSFC00 == 1 )
                case RETURN_QUERY_DATA:
                {
                    for( size_t i = 4; i < xReplyLength - 2; i++ )
                    {
                        if( pucReplyFrame[ i ] != pucRequestFrame[ i ] )
                        {
                            xSetException( ILLEGAL_QUERY_DATA );
                            vSetState( PROCESSING_ERROR );
                            return;
                        }
                    }

                    break;
                }
#endif
#if( configSFC01 == 1 )
                case RESTART_COMMUNICATIONS_OPTION:
#endif
#if( configSFC03 == 1 )
                case CHANGE_ASCII_INPUT_DELIMITER:
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
#if( ( configSFC01 == 1 ) || ( configSFC03 == 1 ) || ( configSFC04 == 1 ) || \
     ( configSFC10 == 1 ) || ( configSFC20 == 1 ) )
                {
                    if( usREPLY_DATA != usREQUEST_DATA )
                    {
                        xSetException( ILLEGAL_OUTPUT_VALUE );
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
                case RETURN_SLAVE_MESSAGE_COUNT:
#endif
#if( configSFC15 == 1 )
                case RETURN_SLAVE_NO_RESPONSE_COUNT:
#endif
#if( configSFC16 == 1 )
                case RETURN_SLAVE_NAK_COUNT:
#endif
#if( configSFC17 == 1 )
                case RETURN_SLAVE_BUSY_COUNT:
#endif
#if( configSFC18 == 1 )
                case RETURN_BUS_CHARACTER_OVERRUN_COUNT:
#endif
#if( ( configSFC02 == 1 ) || ( configSFC11 == 1 ) || ( configSFC12 == 1 ) || \
     ( configSFC13 == 1 ) || ( configSFC14 == 1 ) || ( configSFC15 == 1 ) || \
     ( configSFC16 == 1 ) || ( configSFC17 == 1 ) || ( configSFC18 == 1 ) )
                {
                    if( pxRequest->object != NULL )
                    {
                        *( ( uint16_t * ) pxRequest->object ) = usREPLY_DATA;
                    }

                    break;
                }
#endif
                default:
                {
                    xSetException( ILLEGAL_SUB_FUNCTION );
                    vSetState( PROCESSING_ERROR );
                    return;
                }
            }

            if( pxRequest->action != NULL )
            {
                (*pxRequest->action)();
            }
        }
        else
        {
            xSetException( ILLEGAL_OUTPUT_ADDRESS );
            vSetState( PROCESSING_ERROR );
        }
    }

#endif
/*-----------------------------------------------------------*/

#if( configFC16 == 1 )

    void SerialModbusMaster::vHandlerFC16( void )
    {
        /* Check the response output address */
        if( usREPLY_ADDRESS == usREQUEST_ADDRESS )
        {
            /* Check the response output value */
            if( usREPLY_QUANTITY == usREQUEST_QUANTITY )
            {
                xReplyDataSize = 0;

                if( pxRequest->action != NULL )
                {
                    (*pxRequest->action)();
                }
            }
            else
            {
                xSetException( ILLEGAL_QUANTITY );
                vSetState( PROCESSING_ERROR );
            }
        }
        else
        {
            xSetException( ILLEGAL_OUTPUT_ADDRESS );
            vSetState( PROCESSING_ERROR );
        }
    }

#endif
