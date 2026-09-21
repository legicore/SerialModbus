////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SerialModbusServer.cpp
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

#include "SerialModbusConfig.h"
#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"
#include "SerialModbusServer.h"

#include <Arduino.h>
#if defined( configMB_SERIAL_SW )
    #include <SoftwareSerial.h>
#endif

/*----------------------------------------------------------------------------*/

struct MB_AddressAccess_s
{
    MB_FunctionCode_t xFunctionCode;
    MB_AddrType_t xAddressType;
};

typedef struct MB_AddressAccess_s MB_AddrAccess_t;

/*----------------------------------------------------------------------------*/

static const MB_AddrAccess_t pxAddressAccessMap[] = {

#if( configMB_FC01 == 1 )
    { FC_READ_COILS,                    ADDR_COIL },
#endif
#if( configMB_FC02 == 1 )
    { FC_READ_DISCRETE_INPUTS,          ADDR_DISCRETE_INPUT },
#endif
#if( configMB_FC03 == 1 )
    { FC_READ_HOLDING_REGISTERS,        ADDR_HOLDING_REGISTER },
#endif
#if( configMB_FC04 == 1 )
    { FC_READ_INPUT_REGISTERS,          ADDR_INPUT_REGISTER },
#endif
#if( configMB_FC05 == 1 )
    { FC_WRITE_SINGLE_COIL,             ADDR_COIL },
#endif
#if( configMB_FC06 == 1 )
    { FC_WRITE_SINGLE_REGISTER,         ADDR_HOLDING_REGISTER },
#endif
#if( configMB_FC15 == 1 )
    { FC_WRITE_MULTIPLE_COILS,          ADDR_COIL },
#endif
#if( configMB_FC16 == 1 )
    { FC_WRITE_MULTIPLE_REGISTERS,      ADDR_HOLDING_REGISTER },
#endif
#if( configMB_FC22 == 1 )
    { FC_MASK_WRITE_REGISTER,           ADDR_HOLDING_REGISTER },
#endif
#if( configMB_FC23 == 1 )
    { FC_READ_WRITE_MULTIPLE_REGISTERS, ADDR_HOLDING_REGISTER },
#endif

    /* Marks the end of the map. */
    { ( MB_FunctionCode_t ) 0x00, ADDR_NO_DATA }
};
/*----------------------------------------------------------------------------*/

SerialModbusServer::SerialModbusServer()
{
    xState = SERVER_IDLE;

    #if( configMB_FC08 == 1 )
    {
        vClearDiagnosticCounters();
        usDiagnosticRegister = 0x0000;
    }
    #endif

    bListenOnlyMode = false;

    pxAddressMap = NULL;
    xAddressMapIndex = 0;

    ucServerId = configMB_ID_SERVER_MAX;

    #if( configMB_SERVER_MULTI_ID == 1 )
    {
        xIdCount = 0;
    }
    #endif

    xAddressMapSize = 0;

    bAddressMapLock_sAPI = false;
    bAddressMapLock = false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::begin( uint8_t id, uint32_t baud, MB_Serial_t * serial, uint32_t config )
{
    if( ( id == 0 ) || ( id > configMB_ID_SERVER_MAX ) )
    {
        return false;
    }

    ucServerId = id;

    return SerialModbusBase::begin( baud, serial, config );
}
/*----------------------------------------------------------------------------*/

#if defined( configMB_SERIAL_SW )

    bool SerialModbusServer::begin( uint8_t id, uint32_t baud, MB_SWSerial_t * serial )
    {
        if( ( id == 0 ) || ( id > configMB_ID_SERVER_MAX ) )
        {
            return false;
        }

        ucServerId = id;

        return SerialModbusBase::begin( baud, serial );
    }

#endif
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vSetState( MB_ServerState_t xStatePar )
{
    xState = xStatePar;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusServer::xSetException( MB_Exception_t xException )
{
    if( xException != MB_OK )
    {
        xExceptionStatus = xException;
    }

    return SerialModbusBase::xSetException( xException );
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setAddressMap( MB_Address_t * addressMap )
{
    if( addressMap == NULL )
    {
        if( bAddressMapLock_sAPI == true )
        {
            bAddressMapLock_sAPI = false;
            if( pxAddressMap != NULL )
            {
                free( pxAddressMap );
            }
        }

        pxAddressMap = NULL;
        bAddressMapLock = false;

        return true;
    }
    else /* if( addressMap != NULL ) */
    {
        if( ( pxAddressMap == NULL ) && ( bAddressMapLock_sAPI == false ) )
        {
            pxAddressMap = addressMap;
            bAddressMapLock = true;

            return true;
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::resetAddressMap( void )
{
    return setAddressMap( NULL );
}
/*----------------------------------------------------------------------------*/

MB_Address_t * SerialModbusServer::getAddressMap( void )
{
    return pxAddressMap;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusServer::checkAddressMap( void )
{
    size_t a = 0;
    size_t b = 0;

    if( pxAddressMap != NULL )
    {
        /* Count the number of entries in the address map and make sure that the
         * data field is defined correctly. */
        while( IS_ADDRESS_MAP_END( pxAddressMap[ a ] ) != true )
        {
            if( pxAddressMap[ a ].data == NULL )
            {
                return MB_SERVER_ADDR_DATA;
            }

            if( pxAddressMap[ a ].dataSize == 0 )
            {
                return MB_SERVER_ADDR_DATA_SIZE;
            }

            a++;
        }

        if( a == 1 )
        {
            /* If there is only one entry in the address map, there is no need
             * to perform a overlap check. */
            return MB_OK;
        }
        else if( a > 1 )
        {
            for( --a; a > 0; a-- )
            {
                for( b = 0; b < a; b++ )
                {
                    if( ( &pxAddressMap[ a ] != &pxAddressMap[ b ] ) &&
                        ( pxAddressMap[ a ].addressType == pxAddressMap[ b ].addressType ) )
                    {
                        /* Simplified representation of the algorithm :
                         * ( b[ 0 ] > a[ n-1 ] ) NOR ( a[ 0 ] > b[ n-1 ] ) */
                        if( !( ( pxAddressMap[ b ].address > ( pxAddressMap[ a ].address + pxAddressMap[ a ].dataSize - 1 ) ) ||
                               ( pxAddressMap[ a ].address > ( pxAddressMap[ b ].address + pxAddressMap[ b ].dataSize - 1 ) ) ) )
                        {
                            return MB_SERVER_ADDR_OVERLAP;
                        }
                    }
                }
            }

            return MB_OK;
        }
    }

    return MB_NOK;
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 1 )
    
    void SerialModbusServer::vSetIdMap( void )
    {
        bool bIdFound = false;

        xIdCount = 0;

        for( size_t i = 0; IS_ADDRESS_MAP_END( pxAddressMap[ i ] ) != true; i++ )
        {
            for( size_t j = 0; j < xIdCount; j++ )
            {
                if( ucIdMap[ j ] == pxAddressMap[ i ].id )
                {
                    bIdFound = true;
                }
            }

            if( bIdFound == false )
            {
                ucIdMap[ xIdCount++ ] = pxAddressMap[ i ].id;
            }

            bIdFound = false;
        }
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bCheckId( uint8_t ucId )
{
    #if( configMB_SERVER_MULTI_ID == 1 )
    {
        for( size_t i = 0; i < xIdCount; i++ )
        {
            if( ucId == ucIdMap[ i ] )
            {
                ucServerId = ucId;
                return true;
            }
        }
    }
    #else
    {
        if( ucId == ucServerId )
        {
            return true;
        }
    }
    #endif

    return false;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusServer::process( void )
{
    ( void ) xSetException( MB_OK );

    do
    {
        /* Get the current state and select the associated action. */
        switch( xState )
        {
            case SERVER_IDLE:
            {
                if( xReplyLength >= configMB_FRAME_LEN_MIN )
                {
                    #if( configMB_MODE == configMB_MODE_ASCII )
                    {
                        /* We are in ASCII mode, so we convert the frame to the
                         * ASCII format (this also updates the pdu length). */
                        ( void ) xRtuToAscii( pucReplyFrame, &xReplyLength );
                    }
                    #endif

                    ( void ) xSendData( pucReplyFrame, xReplyLength );

                    vClearReplyFrame();
                }
                else
                {
                    if( xRequestLength < configMB_FRAME_LEN_MAX )
                    {
                        if( bReceiveByte( pucRequestFrame, &xRequestLength ) == true )
                        {
                            #if( configMB_MODE == configMB_MODE_RTU )
                            {
                                vStartInterFrameDelay();
                                vStartInterCharacterTimeout();
                            }
                            #endif

                            #if( configMB_MODE == configMB_MODE_ASCII )
                            {
                                if( pucRequestFrame[ 0 ] != ( uint8_t ) ':' )
                                {
                                    vClearRequestFrame();
                                }
                            }
                            #endif

                            break;
                        }
                    }
                    else
                    {
                        /* Receive buffer overflow -> Increment the bus
                         * charakter overrun counter. */
                        vIncCPT8();
                        ( void ) xSetException( MB_CHARACTER_OVERRUN );

                        /* We go directly back to the idle state and don't send
                         * any reply because it would cause bus collisions if
                         * every server sends an error reply. */
                        vClearRequestFrame();
                        vSetState( SERVER_IDLE );

                        break;
                    }
                }

                /* Check if the start of a frame has been received. */
                if( xRequestLength >= configMB_FRAME_LEN_MIN )
                {
                    #if( configMB_MODE == configMB_MODE_RTU )
                    {
                        if( bTimeoutInterCharacterTimeout() == true )
                        {
                            if( xCheckChecksum( pucRequestFrame, xRequestLength ) == MB_OK )
                            {
                                /* Received a new valid request -> Increment the
                                 * bus message counter. */
                                vIncCPT1();

                                /* Check if the received request is dedicated to
                                 * us or if it is a broadcast. */
                                if( ( bCheckId( ucREQUEST_ID ) == true ) ||
                                    ( ucREQUEST_ID == configMB_ID_BROADCAST ) )
                                {
                                    while( bTimeoutInterFrameDelay() != true );

                                    vSetState( CHECKING_REQUEST );
                                    break;
                                }
                            }
                            else
                            {
                                /* Checksum error -> Increment the bus
                                 * communication error counter. */
                                vIncCPT2();
                            }

                            /* We go directly back to the idle state and don't
                             * send any error reply because it would cause bus
                             * collisions if every server sends one. */
                            vClearRequestFrame();
                            vSetState( SERVER_IDLE );
                        }
                    }
                    #endif

                    #if( configMB_MODE == configMB_MODE_ASCII )
                    {
                        /* Check for the end of the ASCII frame which is marked
                         * by a carriage-return ('\r') followed by a variable
                         * input delimiter (default: line-feed/'\n'). */
                        if( pucRequestFrame[ xRequestLength - 1 ] == ( uint8_t ) cAsciiInputDelimiter )
                        {
                            if( pucRequestFrame[ xRequestLength - 2 ] == ( uint8_t ) '\r' )
                            {
                                /* Convert the request frame from ASCII to RTU
                                 * format and update the request length. */
                                ( void ) xAsciiToRtu( pucRequestFrame, &xRequestLength );

                                if( xCheckChecksum( pucRequestFrame, xRequestLength ) == MB_OK )
                                {
                                    /* Received a new valid request ->
                                     * Increment the bus message counter. */
                                    vIncCPT1();

                                    /* Check if the request is dedicated to us
                                     * or if it is a broadcast. */
                                    if( ( bCheckId( ucREQUEST_ID ) == true ) ||
                                        ( ucREQUEST_ID == configMB_ID_BROADCAST ) )
                                    {
                                        vSetState( CHECKING_REQUEST );
                                        break;
                                    }
                                }
                                else
                                {
                                    /* Checksum error -> Increment the bus
                                     * communication error counter. */
                                    vIncCPT2();
                                }

                                /* We go directly back to the idle state because
                                 * we don't want to send any kind of reply. It
                                 * would cause bus collisions if every server
                                 * sends an error reply. */
                                vClearRequestFrame();
                                vSetState( SERVER_IDLE );
                            }
                        }
                    }
                    #endif
                }

                break;
            }

            case CHECKING_REQUEST:
            {
                /* We received a valid request that is a broadcast or is
                 * addressed to the Id of this deveice -> Increment the server
                 * message counter. */
                vIncCPT4();

                if( xCheckRequest( usREQUEST_ADDRESS, ucREQUEST_FUNCTION_CODE ) == MB_OK )
                {
                    vSetState( PROCESSING_REQUIRED_ACTION );
                }
                else
                {
                    vSetState( FORMATTING_ERROR_REPLY );
                }

                break;
            }

            case PROCESSING_REQUIRED_ACTION:
            {
#if( configMB_FC08 == 1 )
                /* If the Listen Only Mode is active we monitor all bus
                 * messages, but we perform no data processing. Only a request
                 * with function code 8 (MB_DIAGNOSTIC) and sub function code 1
                 * (MB_RESTART_COMMUNICATIONS_OPTION) will be processed, because
                 * it is needed to deactivate the only listen mode. */
                if( ( bListenOnlyMode == false ) ||
                    ( ( ucREQUEST_FUNCTION_CODE == FC_DIAGNOSTIC ) &&
                      ( usREQUEST_SUB_FUNCTION_CODE == SFC_RESTART_COMMUNICATIONS_OPTION ) ) )
                {
#endif
                    switch( ucREQUEST_FUNCTION_CODE )
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
#if( configMB_FC07 == 1 )
                        case FC_READ_EXCEPTION_STATUS :
                        {
                            vHandlerFC07();
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
                            #if( configMB_EXT_EXCEPTION_CODES == 1 )
                            {
                                ( void ) xSetException( MB_SERVER_ILLEGAL_FUNCTION );
                            }
                            #else
                            {
                                ( void ) xSetException( MB_ILLEGAL_FUNCTION );
                            }
                            #endif
                        }
                    }

                    if( ucREQUEST_ID != configMB_ID_BROADCAST )
                    {
                        if( xStatus == MB_OK )
                        {
                            vSetState( FORMATTING_NORMAL_REPLY );
                        }
                        else
                        {
                            vSetState( FORMATTING_ERROR_REPLY );
                        }

                        break;
                    }
                    else
                    {
                        /* This is a broadcast, so we will not send a reply and
                         * increment the no response counter. */
                        vClearReplyFrame();
                        vIncCPT5();
                    }
#if( configMB_FC08 == 1 )
                }
#endif

                vClearRequestFrame();
                vSetState( SERVER_IDLE );

                break;
            }

            case FORMATTING_NORMAL_REPLY:
            {
                ucREPLY_ID = ucServerId;
                ( void ) xSetChecksum( pucReplyFrame, &xReplyLength );

                vClearRequestFrame();
                vSetState( SERVER_IDLE );

                break;
            }

            case FORMATTING_ERROR_REPLY:
            {
                vIncCPT3();

                ucREPLY_ID            = ucServerId;
                ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE | 0x80;
                ucREPLY_ERROR_CODE    = ( uint8_t ) xStatus;

                xReplyLength = 3;

                ( void ) xSetChecksum( pucReplyFrame, &xReplyLength );

                vClearRequestFrame();
                vSetState( SERVER_IDLE );

                break;
            }

            default:
            {
                #if( configMB_EXT_EXCEPTION_CODES == 1 )
                {
                    ( void ) xSetException( MB_SERVER_ILLEGAL_STATE );
                }
                #else
                {
                    ( void ) xSetException( MB_SERVER_DEVICE_FAILURE );
                }
                #endif

                vSetState( FORMATTING_ERROR_REPLY );
            }
        }

        #if( configMB_PROCESS_LOOP_HOOK == 1 )
        {
            /* The process loop hook will only be executed when the state
             * mashine is not in the idle state. Otherwise the loop hook would
             * be execetued with every run through process(). */
            if( ( vProcessLoopHook != NULL ) && ( xState != SERVER_IDLE ) )
            {
                ( vProcessLoopHook )();
            }
        }
        #endif
    }
    while( xState != SERVER_IDLE );

    return xStatus;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusServer::xCheckRequest( uint16_t usReqAddress, uint8_t ucReqFunctionCode )
{
    MB_AddrType_t xReqAddressType = ADDR_NO_DATA;

    /* Do nothing if the address map is not set. */
    if( pxAddressMap == NULL )
    {
        return xSetException( MB_SERVER_DEVICE_FAILURE );
    }

    /* If any Modbus diagnostics are enabled we don't need to do the normal
     * request check on them. All diagnoctic functions are a part of the Modbus
     * protocol (and don't need any definition as a register etc.). */
    if( ( ucREQUEST_FUNCTION_CODE == FC_READ_EXCEPTION_STATUS ) ||
        ( ucREQUEST_FUNCTION_CODE == FC_DIAGNOSTIC            ) ||
        ( ucREQUEST_FUNCTION_CODE == FC_GET_COM_EVENT_COUNTER ) ||
        ( ucREQUEST_FUNCTION_CODE == FC_GET_COM_EVENT_LOG     ) ||
        ( ucREQUEST_FUNCTION_CODE == FC_REPORT_SERVER_ID      ) )
    {
        return MB_OK;
    }

    /* Scan the access map for the address type of the given function code. */
    for( size_t i = 0; pxAddressAccessMap[ i ].xAddressType != ADDR_NO_DATA; i++ )
    {
        if( pxAddressAccessMap[ i ].xFunctionCode == ucReqFunctionCode )
        {
            xReqAddressType = pxAddressAccessMap[ i ].xAddressType;
        }
    }

    if( xReqAddressType == ADDR_NO_DATA )
    {
        /* We could not find the function code in the address access map, so we
         * set the exception and abort the request check. */
        #if( configMB_EXT_EXCEPTION_CODES == 1 )
        {
            return xSetException( MB_SERVER_ILLEGAL_FUNCTION );
        }
        #else
        {
            return xSetException( MB_ILLEGAL_FUNCTION );
        }
        #endif
    }

    /* Reset the address map index. */
    xAddressMapIndex = 0;

    /* Scan the address map and check if the request address value lies in the
     * range of one of the mapped register entries. */
    for( ; IS_ADDRESS_MAP_END( pxAddressMap[ xAddressMapIndex ] ) != true; xAddressMapIndex++ )
    {
#if( configMB_SERVER_MULTI_ID == 1 )
        if( pxAddressMap[ xAddressMapIndex ].id == ucServerId )
        {
#endif
            if( pxAddressMap[ xAddressMapIndex ].addressType == xReqAddressType )
            {
                if( ( usReqAddress >= pxAddressMap[ xAddressMapIndex ].address ) &&
                    ( usReqAddress < ( pxAddressMap[ xAddressMapIndex ].address + ( uint16_t ) pxAddressMap[ xAddressMapIndex ].dataSize ) ) )
                {
                    /* Reset the exception which was set from start. */
                    return MB_OK;
                }
            }
#if( configMB_SERVER_MULTI_ID == 1 )
        }
#endif
    }

    /* At this point, the requested address is not defined/assigned. */
    return xSetException( MB_ILLEGAL_DATA_ADDRESS );
}
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vHandlerFC03_04( void )
{
    size_t xOffset = 0;

    if( ( usREQUEST_QUANTITY >= 0x0001 ) && ( usREQUEST_QUANTITY <= 0x007D ) )
    {
        xOffset = ( size_t ) usREQUEST_ADDRESS - pxAddressMap[ xAddressMapIndex ].address;

        if( ( ( size_t ) usREQUEST_QUANTITY + xOffset ) <= pxAddressMap[ xAddressMapIndex ].dataSize )
        {
            for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
            {
                pucReplyFrame[ ( i * 2 ) + 3 ] = highByte( ( ( uint16_t * ) pxAddressMap[ xAddressMapIndex ].data )[ i + xOffset ] );
                pucReplyFrame[ ( i * 2 ) + 4 ] =  lowByte( ( ( uint16_t * ) pxAddressMap[ xAddressMapIndex ].data )[ i + xOffset ] );
            }

            ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
            ucREPLY_BYTE_COUNT    = ( uint8_t ) usREQUEST_QUANTITY * 2;

            xReplyLength = ( size_t ) ucREPLY_BYTE_COUNT + 3;

            if( pxAddressMap[ xAddressMapIndex ].callback != NULL )
            {
                ( pxAddressMap[ xAddressMapIndex ].callback )();
            }

            return;
        }

        ( void ) xSetException( MB_ILLEGAL_DATA_ADDRESS );

        return;
    }

    #if( configMB_EXT_EXCEPTION_CODES == 1 )
    {
        ( void ) xSetException( MB_SERVER_ILLEGAL_QUANTITY );
    }
    #else
    {
        ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
    }
    #endif
}
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vHandlerFC05( void )
{
    size_t xOffset = 0;
    size_t xBit = 0;

    if( ( usREQUEST_COIL_VALUE == MB_COIL_ON ) || ( usREQUEST_COIL_VALUE == MB_COIL_OFF ) )
    {
        xOffset = ( size_t ) ( usREQUEST_ADDRESS - pxAddressMap[ xAddressMapIndex ].address ) / 8;
        xBit    = ( size_t ) ( usREQUEST_ADDRESS - pxAddressMap[ xAddressMapIndex ].address ) % 8;

        if( usREQUEST_COIL_VALUE == MB_COIL_ON )
        {
            bitSet( ( ( uint8_t * ) pxAddressMap[ xAddressMapIndex ].data )[ xOffset ], xBit );
        }
        else
        {
            bitClear( ( ( uint8_t * ) pxAddressMap[ xAddressMapIndex ].data )[ xOffset ], xBit );
        }

        ucREPLY_FUNCTION_CODE   = ucREQUEST_FUNCTION_CODE;
        ucREPLY_ADDRESS_HI      = ucREQUEST_ADDRESS_HI;
        ucREPLY_ADDRESS_LO      = ucREQUEST_ADDRESS_LO;
        ucREPLY_OUTPUT_VALUE_HI = ucREQUEST_OUTPUT_VALUE_HI;
        ucREPLY_OUTPUT_VALUE_LO = ucREQUEST_OUTPUT_VALUE_LO;

        xReplyLength = 6;

        if( pxAddressMap[ xAddressMapIndex ].callback != NULL )
        {
            ( pxAddressMap[ xAddressMapIndex ].callback )();
        }

        return;
    }

    #if( configMB_EXT_EXCEPTION_CODES == 1 )
    {
        ( void ) xSetException( MB_SERVER_ILLEGAL_COIL_VALUE );
    }
    #else
    {
        ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
    }
    #endif
}
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vHandlerFC06( void )
{
    size_t xOffset = ( size_t ) usREQUEST_ADDRESS - pxAddressMap[ xAddressMapIndex ].address;

    ( ( uint16_t * ) pxAddressMap[ xAddressMapIndex ].data )[ xOffset ] = usREQUEST_REGISTER_VALUE;

    ucREPLY_FUNCTION_CODE     = ucREQUEST_FUNCTION_CODE;
    ucREPLY_ADDRESS_HI        = ucREQUEST_ADDRESS_HI;
    ucREPLY_ADDRESS_LO        = ucREQUEST_ADDRESS_LO;
    ucREPLY_REGISTER_VALUE_HI = ucREQUEST_REGISTER_VALUE_HI;
    ucREPLY_REGISTER_VALUE_LO = ucREQUEST_REGISTER_VALUE_LO;

    xReplyLength = 6;

    if( pxAddressMap[ xAddressMapIndex ].callback != NULL )
    {
        ( pxAddressMap[ xAddressMapIndex ].callback )();
    }
}
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vHandlerFC07( void )
{
    ucREPLY_FUNCTION_CODE    = ucREQUEST_FUNCTION_CODE;
    ucREPLY_EXCEPTION_STATUS = ( uint8_t ) xExceptionStatus;

    xReplyLength = 3;

    if( pxAddressMap[ xAddressMapIndex ].callback != NULL )
    {
        ( pxAddressMap[ xAddressMapIndex ].callback )();
    }
}
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vHandlerFC08( void )
{
    /* Set the common reply data for all MB_DIAGNOSTIC sub functions. */
    ucREPLY_FUNCTION_CODE        = ucREQUEST_FUNCTION_CODE;
    ucREPLY_SUB_FUNCTION_CODE_HI = ucREQUEST_SUB_FUNCTION_CODE_HI;
    ucREPLY_SUB_FUNCTION_CODE_LO = ucREQUEST_SUB_FUNCTION_CODE_LO;

    /* Some of the diagnoostic sub functions just return the received
     * request data (which is in most cases 0x0000). So we apply this data
     * directly at the beginning of the handler and will change it only in the
     * specific cases - the same goes for the reply length. */
    ucREPLY_DATA_HI = ucREQUEST_DATA_HI;
    ucREPLY_DATA_LO = ucREQUEST_DATA_LO;

    xReplyLength = 6;

    switch( usREQUEST_SUB_FUNCTION_CODE )
    {

#if( configMB_SFC00 == 1 )

        case SFC_RETURN_QUERY_DATA:
        {
            xReplyLength = 4;

            for( ; xReplyLength < ( xRequestLength - xChecksumLength ); xReplyLength++ )
            {
                pucReplyFrame[ xReplyLength ] = pucRequestFrame[ xReplyLength ];
            }

            break;
        }

#endif
#if( configMB_SFC01 == 1 )

        case SFC_RESTART_COMMUNICATIONS_OPTION:
        {
            if( ( usREQUEST_DATA == 0x0000 ) || ( usREQUEST_DATA == MB_CLEAR_COM_EVENT_LOG ) )
            {
                /* INFO: The Modbus spec prescribes here that the serial port
                 * must be initialized and restarted, but we don't do that. */

                vClearDiagnosticCounters();

                /* Reset the Only Listen Mode. */
                bListenOnlyMode = false;

                if( usREQUEST_DATA == MB_CLEAR_COM_EVENT_LOG )
                {
                    /* INFO: The Modbus spec prescribes here to clear the
                     * communication event log, which is not implemented. */
                }

                /* INFO: The Modbus spec prescribes here to perform a complete
                 * restart of the device, but we skip that. */
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC02 == 1 )

        case SFC_RETURN_DIAGNOSTIC_REGISTER:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usDiagnosticRegister );
                ucREPLY_DATA_LO =  lowByte( usDiagnosticRegister );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC03 == 1 )

        case SFC_CHANGE_ASCII_INPUT_DELIMITER:
        {
            if( ( bCheckAsciiInputDelimiter( ( char ) ucREQUEST_INPUT_DELIMITER_HI ) == true ) &&
                ( ucREQUEST_INPUT_DELIMITER_LO == 0x00 ) )
            {
                cAsciiInputDelimiter = ( char ) ucREQUEST_INPUT_DELIMITER_HI;

                ucREPLY_INPUT_DELIMITER_HI = ( uint8_t ) cAsciiInputDelimiter;
                ucREPLY_INPUT_DELIMITER_LO = 0x00;
            }
            else
            {
                #if( configMB_EXT_EXCEPTION_CODES == 1 )
                {
                    ( void ) xSetException( MB_SERVER_ILLEGAL_INPUT_DELIMITER );
                }
                #else
                {
                    ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
                }
                #endif

                return;
            }

            break;
        }

#endif
#if( configMB_SFC04 == 1 )

        case SFC_FORCE_LISTEN_ONLY_MODE:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                bListenOnlyMode = true;
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC10 == 1 )

        case SFC_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                vClearDiagnosticCounters();
                diagRegClear();

                xReplyLength = xRequestLength;
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC11 == 1 )

        case SFC_RETURN_BUS_MESSAGE_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usBusMessageCount );
                ucREPLY_DATA_LO =  lowByte( usBusMessageCount );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC12 == 1 )

        case SFC_RETURN_BUS_COMMUNICATION_ERROR_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usBusCommunicationErrorCount );
                ucREPLY_DATA_LO =  lowByte( usBusCommunicationErrorCount );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC13 == 1 )

        case SFC_RETURN_BUS_EXCEPTION_ERROR_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerExceptionErrorCount );
                ucREPLY_DATA_LO =  lowByte( usServerExceptionErrorCount );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC14 == 1 )

        case SFC_RETURN_SERVER_MESSAGE_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerMessageCount );
                ucREPLY_DATA_LO =  lowByte( usServerMessageCount );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC15 == 1 )

        case SFC_RETURN_SERVER_NO_RESPONSE_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerNoResponseCount );
                ucREPLY_DATA_LO =  lowByte( usServerNoResponseCount );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC16 == 1 )

        case SFC_RETURN_SERVER_NAK_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerNAKCount );
                ucREPLY_DATA_LO =  lowByte( usServerNAKCount );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC17 == 1 )

        case SFC_RETURN_SERVER_BUSY_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerBusyCount );
                ucREPLY_DATA_LO =  lowByte( usServerBusyCount );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC18 == 1 )

        case SFC_RETURN_BUS_CHARACTER_OVERRUN_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usBusCharacterOverrunCount );
                ucREPLY_DATA_LO =  lowByte( usBusCharacterOverrunCount );
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif
#if( configMB_SFC20 == 1 )

        case SFC_CLEAR_OVERRUN_COUNTER_AND_FLAG:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                usBusCharacterOverrunCount = 0;

                /* INFO: The Modbus spec prescribes here to also clear an error
                 * flag, but this flag is nowhere specified. */
            }
            else
            {
                ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
            }

            break;
        }

#endif

        default:
        {
            #if( configMB_EXT_EXCEPTION_CODES == 1 )
            {
                ( void ) xSetException( MB_SERVER_ILLEGAL_SUB_FUNCTION );
            }
            #else
            {
                ( void ) xSetException( MB_ILLEGAL_FUNCTION );
            }
            #endif
        }
    }

    if( xStatus == MB_OK )
    {
        if( pxAddressMap[ xAddressMapIndex ].callback != NULL )
        {
            ( pxAddressMap[ xAddressMapIndex ].callback )();
        }
    }
}
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vClearDiagnosticCounters( void )
{
    usBusMessageCount            = 0;
    usBusCommunicationErrorCount = 0;
    usServerExceptionErrorCount  = 0;
    usServerMessageCount         = 0;
    usServerNoResponseCount      = 0;
    usServerNAKCount             = 0;
    usServerBusyCount            = 0;
    usBusCharacterOverrunCount   = 0;
}
/*----------------------------------------------------------------------------*/

uint16_t SerialModbusServer::diagRegGet( void )
{
    return usDiagnosticRegister;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::diagRegGet( size_t bit )
{
    if( bit <= 15 )
    {
        if( bitRead( usDiagnosticRegister, bit ) == 1 )
        {
            return true;
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::diagRegSet( size_t bit )
{
    if( bit <= 15 )
    {
        bitSet( usDiagnosticRegister, bit );
        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::diagRegClear( size_t bit )
{
    if( bit <= 15 )
    {
        bitClear( usDiagnosticRegister, bit );
        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

void SerialModbusServer::diagRegClear( void )
{
    usDiagnosticRegister = 0x0000;
}
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vHandlerFC16( void )
{
    size_t xOffset = 0;

    if( ( usREQUEST_QUANTITY >= 0x0001 ) && ( usREQUEST_QUANTITY <= 0x007B ) )
    {
        if( ucREQUEST_BYTE_COUNT_FC16 == ( ( uint8_t ) usREQUEST_QUANTITY * 2 ) )
        {
            xOffset = ( size_t ) ( usREQUEST_ADDRESS - pxAddressMap[ xAddressMapIndex ].address );

            if( ( ( size_t ) usREQUEST_QUANTITY + xOffset ) <= pxAddressMap[ xAddressMapIndex ].dataSize )
            {
                for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
                {
                    ( ( uint16_t * ) pxAddressMap[ xAddressMapIndex ].data )[ i + xOffset ] = usRequestWord( i, 7 );
                }

                ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
                ucREPLY_ADDRESS_HI    = ucREQUEST_ADDRESS_HI;
                ucREPLY_ADDRESS_LO    = ucREQUEST_ADDRESS_LO;
                ucREPLY_QUANTITY_HI   = ucREQUEST_QUANTITY_HI;
                ucREPLY_QUANTITY_LO   = ucREQUEST_QUANTITY_LO;

                xReplyLength = 6;

                if( pxAddressMap[ xAddressMapIndex ].callback != NULL )
                {
                    ( pxAddressMap[ xAddressMapIndex ].callback )();
                }

                return;
            }

            ( void ) xSetException( MB_ILLEGAL_DATA_ADDRESS );

            return;
        }
    }

    #if( configMB_EXT_EXCEPTION_CODES == 1 )
    {
        ( void ) xSetException( MB_SERVER_ILLEGAL_QUANTITY );
    }
    #else
    {
        ( void ) xSetException( MB_ILLEGAL_DATA_VALUE );
    }
    #endif
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bCreateAddress( uint8_t id, MB_AddrType_t addressType, uint16_t address, size_t dataSize, MB_Callback_f callback )
{
    MB_Address_t * pxAddressMapTmp = NULL;

    if( ( id == configMB_ID_BROADCAST ) || ( id > configMB_ID_SERVER_MAX ) ||
        ( dataSize == 0 ) ||
        ( address > ( address + ( uint16_t ) dataSize - 1 ) ) ||
        ( bFindAddress( id, addressType, address ) == true ) ||
        ( bFindAddress( id, addressType, address + ( uint16_t ) dataSize - 1 ) == true ) ||
        ( bAddressMapLock == true ) )
    {
        return false;
    }

    if( pxAddressMap == NULL )
    {
        pxAddressMapTmp = ( MB_Address_t * ) malloc( sizeof( MB_Address_t ) * 2 );
        if( pxAddressMapTmp != NULL )
        {
            ( void ) bClearAddressMapEntry( &pxAddressMapTmp[ 0 ] );
            xAddressMapSize = 1;
        }
    }
    else
    {
        pxAddressMapTmp = ( MB_Address_t * ) realloc( pxAddressMap, sizeof( MB_Address_t ) * ( xAddressMapSize + 1 ) );
    }

    if( pxAddressMapTmp != NULL )
    {
        if( ( addressType == ADDR_COIL ) || ( addressType == ADDR_DISCRETE_INPUT ) )
        {
            pxAddressMapTmp[ xAddressMapSize - 1 ].data = ( uint8_t * ) calloc( mbBITS_TO_BYTES( dataSize ), sizeof( uint8_t ) );
        }
        else /* if( ( addressType == ADDR_HOLDING_REGISTER ) || ( addressType == ADDR_INPUT_REGISTER ) ) */
        {
            pxAddressMapTmp[ xAddressMapSize - 1 ].data = ( uint16_t * ) calloc( dataSize, sizeof( uint16_t ) );
        }

        if( pxAddressMapTmp[ xAddressMapSize - 1 ].data != NULL )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
            pxAddressMapTmp[ xAddressMapSize - 1 ].id = id;
#endif
            pxAddressMapTmp[ xAddressMapSize - 1 ].addressType = addressType;
            pxAddressMapTmp[ xAddressMapSize - 1 ].address = address;
            pxAddressMapTmp[ xAddressMapSize - 1 ].dataSize = dataSize;
            pxAddressMapTmp[ xAddressMapSize - 1 ].callback = callback;

            ( void ) bClearAddressMapEntry( &pxAddressMapTmp[ xAddressMapSize ] );
            pxAddressMap = pxAddressMapTmp;
            bAddressMapLock_sAPI = true;
            xAddressMapSize += 1;

            #if( configMB_SERVER_MULTI_ID == 1 )
            {
                /* In case a new ID gets introduced, we automatically reset the
                 * ID map after every successful register creation. */
                vSetIdMap();
            }
            #endif

            return true;
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bGetAddressBit( uint8_t id, MB_AddrType_t addressType, uint16_t address, uint16_t * data )
{
    size_t xOffset = 0;
    size_t xBit = 0;

    if( ( id != 0 ) && ( id <= configMB_ID_SERVER_MAX ) &&
        ( ( addressType == ADDR_COIL ) || ( addressType == ADDR_DISCRETE_INPUT ) ) &&
        ( data != NULL ) &&
        ( pxAddressMap != NULL ) )
    {
        for( size_t i = 0; IS_ADDRESS_MAP_END( pxAddressMap[ i ] ) != true; i++ )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
            if( id == pxAddressMap[ i ].id )
            {
#endif
                if( ( address >= pxAddressMap[ i ].address ) &&
                    ( address < ( pxAddressMap[ i ].address + ( uint16_t ) pxAddressMap[ i ].dataSize ) ) &&
                    ( pxAddressMap[ i ].addressType == addressType ) )
                {
                    xOffset = ( address - pxAddressMap[ i ].address ) / 8;
                    xBit    = ( address - pxAddressMap[ i ].address ) % 8;

                    if( bitRead( ( ( uint8_t * ) pxAddressMap[ i ].data )[ xOffset ], xBit ) == 1 )
                    {
                        *data = MB_COIL_ON;
                    }
                    else
                    {
                        *data = MB_COIL_OFF;
                    }

                    return true;
                }
#if( configMB_SERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bSetAddressBit( uint8_t id, MB_AddrType_t addressType, uint16_t address, uint16_t value )
{
    size_t xOffset = 0;
    size_t xBit = 0;

    if( ( id != 0 ) && ( id <= configMB_ID_SERVER_MAX ) &&
        ( ( addressType == ADDR_COIL ) || ( addressType == ADDR_DISCRETE_INPUT ) ) &&
        ( ( value == 0 ) || ( value == 1 ) || ( value == MB_COIL_ON ) ) &&
        ( pxAddressMap != NULL ) )
    {
        for( size_t i = 0; IS_ADDRESS_MAP_END( pxAddressMap[ i ] ) != true; i++ )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
            if( id == pxAddressMap[ i ].id )
            {
#endif
                if( ( address >= pxAddressMap[ i ].address ) &&
                    ( address < ( pxAddressMap[ i ].address + ( uint16_t ) pxAddressMap[ i ].dataSize ) ) &&
                    ( pxAddressMap[ i ].addressType == addressType ) )
                {
                    xOffset = ( address - pxAddressMap[ i ].address ) / 8;
                    xBit    = ( address - pxAddressMap[ i ].address ) % 8;

                    if( ( value == 1 ) || ( value == MB_COIL_ON ) )
                    {
                        bitSet( ( ( uint8_t * ) pxAddressMap[ i ].data )[ xOffset ], xBit );
                    }
                    else
                    {
                        bitClear( ( ( uint8_t * ) pxAddressMap[ i ].data )[ xOffset ], xBit );
                    }

                    return true;
                }
#if( configMB_SERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bGetAddressWord( uint8_t id, MB_AddrType_t addressType, uint16_t address, uint16_t * data )
{
    size_t xOffset = 0;

    if( ( id != 0 ) && ( id <= configMB_ID_SERVER_MAX ) &&
        ( ( addressType == ADDR_HOLDING_REGISTER ) || ( addressType == ADDR_INPUT_REGISTER ) ) &&
        ( data != NULL ) &&
        ( pxAddressMap != NULL ) )
    {
        for( size_t i = 0; IS_ADDRESS_MAP_END( pxAddressMap[ i ] ) != true; i++ )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
            if( id == pxAddressMap[ i ].id )
            {
#endif
                if( ( address >= pxAddressMap[ i ].address ) &&
                    ( address < ( pxAddressMap[ i ].address + ( uint16_t ) pxAddressMap[ i ].dataSize ) ) &&
                    ( pxAddressMap[ i ].addressType == addressType ) )
                {
                    xOffset = address - pxAddressMap[ i ].address;
                    *data = ( ( uint16_t * ) pxAddressMap[ i ].data )[ xOffset ];

                    return true;
                }
#if( configMB_SERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bSetAddressWord( uint8_t id, MB_AddrType_t addressType, uint16_t address, uint16_t value )
{
    size_t xOffset = 0;

    if( ( id != 0 ) && ( id <= configMB_ID_SERVER_MAX ) &&
        ( ( addressType == ADDR_HOLDING_REGISTER ) || ( addressType == ADDR_INPUT_REGISTER ) ) &&
        ( pxAddressMap != NULL ) )
    {
        for( size_t i = 0; IS_ADDRESS_MAP_END( pxAddressMap[ i ] ) != true; i++ )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
            if( id == pxAddressMap[ i ].id )
            {
#endif
                if( ( address >= pxAddressMap[ i ].address ) &&
                    ( address < ( pxAddressMap[ i ].address + ( uint16_t ) pxAddressMap[ i ].dataSize ) ) &&
                    ( pxAddressMap[ i ].addressType == addressType ) )
                {
                    xOffset = address - pxAddressMap[ i ].address;
                    ( ( uint16_t * ) pxAddressMap[ i ].data )[ xOffset ] = value;

                    return true;
                }
#if( configMB_SERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::createCoil( uint16_t address, size_t dataSize, MB_Callback_f callback )
    {
        return createCoil( configMB_ID_SERVER_MAX, address, dataSize, callback );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::createCoil( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback )
{
    return bCreateAddress( id, ADDR_COIL, address, dataSize, callback );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::getCoil( uint16_t address, uint16_t * data )
    {
        return getCoil( configMB_ID_SERVER_MAX, address, data );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::getCoil( uint8_t id, uint16_t address, uint16_t * data )
{
    return bGetAddressBit( id, ADDR_COIL, address, data );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::setCoil( uint16_t address, uint16_t value )
    {
        return setCoil( configMB_ID_SERVER_MAX, address, value );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setCoil( uint8_t id, uint16_t address, uint16_t value )
{
    return bSetAddressBit( id, ADDR_COIL, address, value );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::createDiscreteInput( uint16_t address, size_t dataSize, MB_Callback_f callback )
    {
        return createDiscreteInput( configMB_ID_SERVER_MAX, address, dataSize, callback );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::createDiscreteInput( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback )
{
    return bCreateAddress( id, ADDR_DISCRETE_INPUT, address, dataSize, callback );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::getDiscreteInput( uint16_t address, uint16_t * data )
    {
        return getDiscreteInput( configMB_ID_SERVER_MAX, address, data );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::getDiscreteInput( uint8_t id, uint16_t address, uint16_t * data )
{
    return bGetAddressBit( id, ADDR_DISCRETE_INPUT, address, data );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::setDiscreteInput( uint16_t address, uint16_t value )
    {
        return setDiscreteInput( configMB_ID_SERVER_MAX, address, value );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setDiscreteInput( uint8_t id, uint16_t address, uint16_t value )
{
    return bSetAddressBit( id, ADDR_DISCRETE_INPUT, address, value );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::createHoldingRegister( uint16_t address, size_t dataSize, MB_Callback_f callback )
    {
        return createHoldingRegister( configMB_ID_SERVER_MAX, address, dataSize, callback );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::createHoldingRegister( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback )
{
    return bCreateAddress( id, ADDR_HOLDING_REGISTER, address, dataSize, callback );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::getHoldingRegister( uint16_t address, uint16_t * data )
    {
        return getHoldingRegister( configMB_ID_SERVER_MAX, address, data );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::getHoldingRegister( uint8_t id, uint16_t address, uint16_t * data )
{
    return bGetAddressWord( id, ADDR_HOLDING_REGISTER, address, data );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::setHoldingRegister( uint16_t address, uint16_t value )
    {
        return setHoldingRegister( configMB_ID_SERVER_MAX, address, value );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setHoldingRegister( uint8_t id, uint16_t address, uint16_t value )
{
    return bSetAddressWord( id, ADDR_HOLDING_REGISTER, address, value );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::createInputRegister( uint16_t address, size_t dataSize, MB_Callback_f callback )
    {
        return createInputRegister( configMB_ID_SERVER_MAX, address, dataSize, callback );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::createInputRegister( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback )
{
    return bCreateAddress( id, ADDR_INPUT_REGISTER, address, dataSize, callback );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::getInputRegister( uint16_t address, uint16_t * data )
    {
        return getInputRegister( configMB_ID_SERVER_MAX, address, data );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::getInputRegister( uint8_t id, uint16_t address, uint16_t * data )
{
    return bGetAddressWord( id, ADDR_INPUT_REGISTER, address, data );
}
/*----------------------------------------------------------------------------*/

#if( configMB_SERVER_MULTI_ID == 0 )

    bool SerialModbusServer::setInputRegister( uint16_t address, uint16_t value )
    {
        return setInputRegister( configMB_ID_SERVER_MAX, address, value );
    }

#endif
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setInputRegister( uint8_t id, uint16_t address, uint16_t value )
{
    return bSetAddressWord( id, ADDR_INPUT_REGISTER, address, value );
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bClearAddressMapEntry( MB_Address_t * pxAddressMapEntry )
{
    if( pxAddressMapEntry != NULL )
    {
#if( configMB_SERVER_MULTI_ID == 1 )
        pxAddressMapEntry->id       = ADDR_MAP_END_ID;
#endif
        pxAddressMapEntry->addressType = ADDR_MAP_END_ADDRESS_TYPE;
        pxAddressMapEntry->address     = ADDR_MAP_END_ADDRESS;
        pxAddressMapEntry->data        = ADDR_MAP_END_DATA;
        pxAddressMapEntry->dataSize    = ADDR_MAP_END_DATA_SIZE;
        pxAddressMapEntry->callback    = ADDR_MAP_END_CALLBACK;

        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bFindAddress( uint8_t ucId, MB_AddrType_t xAddressType, uint16_t usAddress )
{
    if( ( ucId != 0 ) && ( ucId <= configMB_ID_SERVER_MAX ) && ( pxAddressMap != NULL ) )
    {
        for( size_t i = 0; IS_ADDRESS_MAP_END( pxAddressMap[ i ] ) != true; i++ )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
            if( ucId == pxAddressMap[ i ].id )
            {
#endif
                if( pxAddressMap[ i ].addressType == xAddressType )
                {
                    if( ( usAddress >= pxAddressMap[ i ].address ) &&
                        ( usAddress < ( pxAddressMap[ i ].address + ( uint16_t ) pxAddressMap[ i ].dataSize ) ) )
                    {
                        return true;
                    }
                }
#if( configMB_SERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return false;
}
