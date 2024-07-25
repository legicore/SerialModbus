////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusServer.cpp
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
#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"
#include "SerialModbusServer.h"

#include <Arduino.h>
#if defined( COMPAT_SOFTWARE_SERIAL )
    #include <SoftwareSerial.h>
#endif

/*-----------------------------------------------------------*/

struct MBAccessRights_s
{
    unsigned uxAccess       : 2;
    unsigned uxFunctionCode : 6;
};

typedef struct MBAccessRights_s MBAccessRights_t;

static const MBAccessRights_t pxAccessRights[] = {

    /* Bit Data Access */
#if( configFC01 == 1 )
    { RD, READ_COILS },
#endif
#if( configFC02 == 1 )
    { RD, READ_DISCRETE_INPUTS },
#endif
#if( configFC05 == 1 )
    { WR, WRITE_SINGLE_COIL },
#endif
#if( configFC15 == 1 )
    { WR, WRITE_MULTIPLE_COILS },
#endif

    /* Word Data Access */
#if( configFC03 == 1 )
    { RD, READ_HOLDING_REGISTERS },
#endif
#if( configFC04 == 1 )
    { RD, READ_INPUT_REGISTERS },
#endif
#if( configFC06 == 1 )
    { WR, WRITE_SINGLE_REGISTER },
#endif
#if( configFC16 == 1 )
    { WR, WRITE_MULTIPLE_REGISTERS },
#endif
#if( configFC22 == 1 )
    { WR, MASK_WRITE_REGISTER },
#endif
#if( configFC23 == 1 )
    { RW, READ_WRITE_MULTIPLE_REGISTERS },
#endif
#if( configFC24 == 1 )
    { RD, READ_FIFO_QUEUE },
#endif

    /* File Record Data Access */
#if( configFC20 == 1 )
    { RD, READ_FILE_RECORD },
#endif
#if( configFC21 == 1 )
    { WR, WRITE_FILE_RECORD },
#endif

    /* Diagnostics */
#if( configFC07 == 1 )
    { RD, READ_EXCEPTION_STATUS },
#endif
#if( configFC08 == 1 )
    { RW, DIAGNOSTIC },
#endif
#if( configFC11 == 1 )
    { RD, GET_COM_EVENT_COUNTER },
#endif
#if( configFC12 == 1 )
    { RD, GET_COM_EVENT_LOG },
#endif
#if( configFC17 == 1 )
    { RD, REPORT_SERVER_ID },
#endif

    /* Marks the end of the list. */
    { 0b00, 0b000000 }
};
/*-----------------------------------------------------------*/

SerialModbusServer::SerialModbusServer()
{
    xState = SERVER_IDLE;

    #if( configFC08 == 1 )
    {
        vClearDiagnosticCounters();
        usDiagnosticRegister = 0x0000;
    }
    #endif

    bListenOnlyMode = false;

    pxRegisterMap = NULL;
    xRegisterMapIndex = 0;

    ucServerId = configID_SERVER_MAX;

    #if( configSERVER_MULTI_ID == 1 )
    {
        xIdCount = 0;
    }
    #endif

    xRegisterMapSize = 0;
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::begin( uint8_t id, uint32_t baud, Serial_t * serial, uint32_t config )
{
    if( ( id == 0 ) || ( id > configID_SERVER_MAX ) )
    {
        return false;
    }

    ucServerId = id;

    #if( configSERVER_MULTI_ID == 1 )
    {
        vSetIdMap();
    }
    #endif

    return SerialModbusBase::begin( baud, serial, config );
}
/*-----------------------------------------------------------*/

#if defined( COMPAT_SOFTWARE_SERIAL )

    bool SerialModbusServer::begin( uint8_t serverId, uint32_t baud, SoftwareSerial * serial )
    {
        if( ( serverId == 0 ) || ( serverId > configID_SERVER_MAX ) )
        {
            return false;
        }

        ucServerId = serverId;

        return SerialModbusBase::begin( baud, serial );
    }

#endif
/*-----------------------------------------------------------*/

void SerialModbusServer::vSetState( MBServerState_t xStatePar )
{
    xState = xStatePar;
}
/*-----------------------------------------------------------*/

void SerialModbusServer::setRegisterMap( MBRegister_t * registerMap )
{
    pxRegisterMap = registerMap;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusServer::checkRegisterMap( void )
{
    size_t a = 0;
    size_t b = 0;

    if( pxRegisterMap != NULL )
    {
        /* Count the number of entries in the register map. */
        while( pxRegisterMap[ ++a ].data != NULL );
        if( a == 1 )
        {
            /* If there is only one entry in the register map, there is no need
            to perform a check. */
            return OK;
        }
        else if( a > 1 )
        {
            for( --a; a > 0; a-- )
            {
                for( b = 0; b < a; b++ )
                {
                    if( &pxRegisterMap[ a ] != &pxRegisterMap[ b ] )
                    {
                        /* Simplified representation of the algorithm :
                        if( ( a[ n-1 ] < b[ 0 ] ) NOR ( a[ 0 ] > b[ n-1 ] ) ) */
                        if( !( ( ( pxRegisterMap[ a ].address + ( uint16_t ) pxRegisterMap[ a ].dataSize - 1 ) < pxRegisterMap[ b ].address ) ||
                            ( pxRegisterMap[ a ].address > ( pxRegisterMap[ b ].address + ( uint16_t ) pxRegisterMap[ b ].dataSize - 1 ) ) ) )
                        {
                            return NOK;
                        }
                    }
                }
            }

            return OK;
        }
    }

    return NOK;
}
/*-----------------------------------------------------------*/

#if( configSERVER_MULTI_ID == 1 )
    
    void SerialModbusServer::vSetIdMap( void )
    {
        bool bIdFound = false;

        for( size_t i = 0; pxRegisterMap[ i ].data != NULL; i++ )
        {
            for( size_t j = 0; j < xIdCount; j++ )
            {
                if( ucIdMap[ j ] == pxRegisterMap[ i ].id )
                {
                    bIdFound = true;
                }
            }

            if( bIdFound == false )
            {
                ucIdMap[ xIdCount++ ] = pxRegisterMap[ i ].id;
            }

            bIdFound = false;
        }
    }

#endif
/*-----------------------------------------------------------*/

bool SerialModbusServer::bCheckId( uint8_t ucId )
{
    #if( configSERVER_MULTI_ID == 1 )
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
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusServer::process( void )
{
    ( void ) xSetException( OK );

    do
    {
        /* Get the current state and select the associated action. */
        switch( xState )
        {
            case SERVER_IDLE :
            {
                if( xReplyLength >= configFRAME_LEN_MIN )
                {
                    #if( configMODE == configMODE_ASCII )
                    {
                        /* We are in ASCII mode, so we convert the frame to the
                        ASCII format (this also updates the pdu length). */
                        ( void ) xRtuToAscii( pucReplyFrame, &xReplyLength );
                    }
                    #endif

                    ( void ) xSendData( pucReplyFrame, xReplyLength );

                    vClearReplyFrame();
                }
                else
                {
                    if( xRequestLength < configFRAME_LEN_MAX )
                    {
                        if( bReceiveByte( pucRequestFrame, &xRequestLength ) == true )
                        {
                            #if( configMODE == configMODE_RTU )
                            {
                                vStartInterFrameDelay();
                                vStartInterCharacterTimeout();
                            }
                            #endif

                            #if( configMODE == configMODE_ASCII )
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
                        charakter overrun counter. */
                        vIncCPT8();
                        ( void ) xSetException( CHARACTER_OVERRUN );

                        /* We go directly back to the idle state and don't send
                        any reply because it would cause bus collisions if every
                        server sends an error reply. */
                        vClearRequestFrame();
                        vSetState( SERVER_IDLE );

                        break;
                    }
                }

                /* Check if the start of a frame has been received. */
                if( xRequestLength >= configFRAME_LEN_MIN )
                {
                    #if( configMODE == configMODE_RTU )
                    {
                        if( bTimeoutInterCharacterTimeout() == true )
                        {
                            if( xCheckChecksum( pucRequestFrame, xRequestLength ) == OK )
                            {
                                /* Received a new valid request -> Increment the
                                bus message counter. */
                                vIncCPT1();

                                /* Check if the received request is dedicated to
                                us or if it is a broadcast. */
                                if( ( bCheckId( ucREQUEST_ID ) == true ) ||
                                    ( ucREQUEST_ID == configID_BROADCAST ) )
                                {
                                    while( bTimeoutInterFrameDelay() != true );

                                    vSetState( CHECKING_REQUEST );
                                    break;
                                }
                            }
                            else
                            {
                                /* Checksum error -> Increment the bus
                                communication error counter. */
                                vIncCPT2();
                            }

                            /* We go directly back to the idle state and don't
                            send any reply because it would cause bus collisions
                            if every server sends an error reply. */
                            vClearRequestFrame();
                            vSetState( SERVER_IDLE );
                        }
                    }
                    #endif

                    #if( configMODE == configMODE_ASCII )
                    {
                        /* Check for the end of the ASCII frame which is marked
                        by a carriage-return ('\r') followed by a variable input
                        delimiter (default: line-feed/'\n'). */
                        if( pucRequestFrame[ xRequestLength - 1 ] == ( uint8_t ) cAsciiInputDelimiter )
                        {
                            if( pucRequestFrame[ xRequestLength - 2 ] == ( uint8_t ) '\r' )
                            {
                                /* Convert the request frame from ASCII to RTU
                                format and update the request length. */
                                ( void ) xAsciiToRtu( pucRequestFrame, &xRequestLength );

                                if( xCheckChecksum( pucRequestFrame, xRequestLength ) == OK )
                                {
                                    /* Received a new valid request ->
                                    Increment the bus message counter. */
                                    vIncCPT1();

                                    /* Check if the received request is
                                    dedicated to us or if it is a broadcast. */
                                    if( ( bCheckId( ucREQUEST_ID ) == true ) ||
                                        ( ucREQUEST_ID == configID_BROADCAST ) )
                                    {
                                        vSetState( CHECKING_REQUEST );
                                        break;
                                    }
                                }
                                else
                                {
                                    /* Checksum error -> Increment the bus
                                    communication error counter. */
                                    vIncCPT2();
                                }

                                /* We go directly back to the idle state because
                                we don't want to send any kind of reply. It
                                would cause bus collisions if every server
                                sends an error reply. */
                                vClearRequestFrame();
                                vSetState( SERVER_IDLE );
                            }
                        }
                    }
                    #endif
                }

                break;
            }

            case CHECKING_REQUEST :
            {
                /* We received a valid request that is a broadcast or is
                addressed to the Id of this deveice -> Increment the server
                message counter. */
                vIncCPT4();

                if( xCheckRequest( usREQUEST_ADDRESS, ucREQUEST_FUNCTION_CODE ) == OK )
                {
                    vSetState( PROCESSING_REQUIRED_ACTION );
                }
                else
                {
                    vSetState( FORMATTING_ERROR_REPLY );
                }

                break;
            }

            case PROCESSING_REQUIRED_ACTION :
            {
#if( configFC08 == 1 )
                /* If the Listen Only Mode is active we monitor all bus
                messages, but we perform no data processing. Only a request with
                function code 8 (DIAGNOSTIC) and sub function code 1
                (RESTART_COMMUNICATIONS_OPTION) will be processed, because it is
                needed to deactivate the only listen mode. */
                if( ( bListenOnlyMode == false ) ||
                    ( ( ucREQUEST_FUNCTION_CODE == DIAGNOSTIC ) &&
                      ( usREQUEST_SUB_FUNCTION_CODE == RESTART_COMMUNICATIONS_OPTION ) ) )
                {
#endif
                    switch( ucREQUEST_FUNCTION_CODE )
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
                            #if( configEXTENDED_EXCEPTION_CODES == 1 )
                            {
                                ( void ) xSetException( SERVER_ILLEGAL_FUNCTION );
                            }
                            #else
                            {
                                ( void ) xSetException( ILLEGAL_FUNCTION );
                            }
                            #endif
                        }
                    }

                    if( ucREQUEST_ID != configID_BROADCAST )
                    {
                        if( xException == OK )
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
                        /* This is a broadcast, so we clear the reply frame to
                        send no reply and increment the no response counter. */
                        vClearReplyFrame();
                        vIncCPT5();
                    }
#if( configFC08 == 1 )
                }
#endif

                vClearRequestFrame();
                vSetState( SERVER_IDLE );

                break;
            }

            case FORMATTING_NORMAL_REPLY :
            {
                ucREPLY_ID = ucServerId;
                ( void ) xSetChecksum( pucReplyFrame, &xReplyLength );

                vClearRequestFrame();
                vSetState( SERVER_IDLE );

                break;
            }

            case FORMATTING_ERROR_REPLY :
            {
                vIncCPT3();

                ucREPLY_ID            = ucServerId;
                ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE | 0x80;
                ucREPLY_ERROR_CODE    = ( uint8_t ) xException;

                xReplyLength = 3;

                ( void ) xSetChecksum( pucReplyFrame, &xReplyLength );

                vClearRequestFrame();
                vSetState( SERVER_IDLE );

                break;
            }

            default :
            {
                #if( configEXTENDED_EXCEPTION_CODES == 1 )
                {
                    ( void ) xSetException( SERVER_ILLEGAL_STATE );
                }
                #else
                {
                    ( void ) xSetException( SERVER_DEVICE_FAILURE );
                }
                #endif

                vSetState( FORMATTING_ERROR_REPLY );
            }
        }

        #if( configPROCESS_LOOP_HOOK == 1 )
        {
            /* The process loop hook will only be executed when the state
            mashine is not in the idle state. Otherwise the loop hook would be
            execetued with every run through process(). */
            if( ( vProcessLoopHook != NULL ) && ( xState != SERVER_IDLE ) )
            {
                ( vProcessLoopHook )();
            }
        }
        #endif
    }
    while( xState != SERVER_IDLE );

    return xException;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusServer::xCheckRequest( uint16_t usReqAddress, uint8_t ucReqFunctionCode )
{
    /* Do nothing if the register map is not set. */
    if( pxRegisterMap == NULL )
    {
        return NOK;
    }

    /* If the diagnostic functions are enabled we don't need to do the normal
    request check. All diagnoctic functions are a part of the Modbus protocol
    (and don't need any definition as a register etc.). */
    if( ucREQUEST_FUNCTION_CODE == ( uint8_t ) DIAGNOSTIC )
    {
        return OK;
    }

    /* Reset the register map index. */
    xRegisterMapIndex = 0;

    /* Before we find a matching register map entry the exception will be set by
    default. If successful the exception is reset to 'OK' or will be overwritten
    if another error occurs. Otherwise it persists which means that we could not
    find a matching register map entry. */
    ( void ) xSetException( ILLEGAL_DATA_ADDRESS );

    /* Scan the register map and check if the request address value lies in the
    range of one of the mapped register entries. */
    for( ; pxRegisterMap[ xRegisterMapIndex ].data != NULL; xRegisterMapIndex++ )
    {
#if( configSERVER_MULTI_ID == 1 )
        if( pxRegisterMap[ xRegisterMapIndex ].id == ucServerId )
        {
#endif
            if( ( usReqAddress >= pxRegisterMap[ xRegisterMapIndex ].address ) &&
                ( usReqAddress < ( pxRegisterMap[ xRegisterMapIndex ].address + ( uint16_t ) pxRegisterMap[ xRegisterMapIndex ].dataSize ) ) )
            {
                /* Scan the access rights map for the request function code. */
                for( size_t i = 0; pxAccessRights[ i ].uxAccess != NA; i++ )
                {
                    if( ucReqFunctionCode == ( uint8_t ) pxAccessRights[ i ].uxFunctionCode )
                    {
                        /* Check if the type of the request function code has
                        the right to access the destination register. */
                        if( ( pxRegisterMap[ xRegisterMapIndex ].access & ( MBAccess_t ) pxAccessRights[ i ].uxAccess ) != 0 )
                        {
                            /* Reset the exception which was set from start. */
                            return xSetException( OK );
                        }

                        /* While register access rights are not a standard
                        feature of Modbus we will set a non standard exception
                        and abort the for loop. */
                        #if( configEXTENDED_EXCEPTION_CODES == 1 )
                        {
                            ( void ) xSetException( SERVER_ILLEGAL_ACCESS );
                        }
                        #else
                        {
                            ( void ) xSetException( ILLEGAL_FUNCTION );
                        }
                        #endif

                        return NOK;
                    }
                }

                /* We could not find the function code in the access rights map
                so we set the exception and abort the for loop. */
                #if( configEXTENDED_EXCEPTION_CODES == 1 )
                {
                    ( void ) xSetException( SERVER_ILLEGAL_FUNCTION );
                }
                #else
                {
                    ( void ) xSetException( ILLEGAL_FUNCTION );
                }
                #endif

                break;
            }
#if( configSERVER_MULTI_ID == 1 )
        }
#endif
    }

    return NOK;
}
/*-----------------------------------------------------------*/

void SerialModbusServer::vHandlerFC03_04( void )
{
    size_t xOffset = 0;

    if( ( usREQUEST_QUANTITY >= 0x0001 ) && ( usREQUEST_QUANTITY <= 0x007D ) )
    {
        xOffset = ( size_t ) usREQUEST_ADDRESS - pxRegisterMap[ xRegisterMapIndex ].address;

        if( ( ( size_t ) usREQUEST_QUANTITY + xOffset ) <= pxRegisterMap[ xRegisterMapIndex ].dataSize )
        {
            for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
            {
                pucReplyFrame[ ( i * 2 ) + 3 ] = highByte( pxRegisterMap[ xRegisterMapIndex ].data[ i + xOffset ] );
                pucReplyFrame[ ( i * 2 ) + 4 ] =  lowByte( pxRegisterMap[ xRegisterMapIndex ].data[ i + xOffset ] );
            }

            ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
            ucREPLY_BYTE_COUNT    = ( uint8_t ) usREQUEST_QUANTITY * 2;

            xReplyLength = ( size_t ) ucREPLY_BYTE_COUNT + 3;

            if( pxRegisterMap[ xRegisterMapIndex ].callback != NULL )
            {
                ( pxRegisterMap[ xRegisterMapIndex ].callback )();
            }

            return;
        }

        ( void ) xSetException( ILLEGAL_DATA_ADDRESS );

        return;
    }

    #if( configEXTENDED_EXCEPTION_CODES == 1 )
    {
        ( void ) xSetException( SERVER_ILLEGAL_QUANTITY );
    }
    #else
    {
        ( void ) xSetException( ILLEGAL_DATA_VALUE );
    }
    #endif
}
/*-----------------------------------------------------------*/

void SerialModbusServer::vHandlerFC05( void )
{
    if( ( usREQUEST_COIL_VALUE == COIL_ON ) || ( usREQUEST_COIL_VALUE == COIL_OFF ) )
    {
        pxRegisterMap[ xRegisterMapIndex ].data[ 0 ] = usREQUEST_COIL_VALUE;

        ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
        ucREPLY_ADDRESS_HI    = ucREQUEST_ADDRESS_HI;
        ucREPLY_ADDRESS_LO    = ucREQUEST_ADDRESS_LO;
        ucREPLY_COIL_VALUE_HI = ucREQUEST_COIL_VALUE_HI;
        ucREPLY_COIL_VALUE_LO = ucREQUEST_COIL_VALUE_LO;

        xReplyLength = 6;

        if( pxRegisterMap[ xRegisterMapIndex ].callback != NULL )
        {
            ( pxRegisterMap[ xRegisterMapIndex ].callback )();
        }

        return;
    }

    #if( configEXTENDED_EXCEPTION_CODES == 1 )
    {
        ( void ) xSetException( SERVER_ILLEGAL_COIL_VALUE );
    }
    #else
    {
        ( void ) xSetException( ILLEGAL_DATA_VALUE );
    }
    #endif
}
/*-----------------------------------------------------------*/

void SerialModbusServer::vHandlerFC06( void )
{
    size_t xOffset = ( size_t ) usREQUEST_ADDRESS - pxRegisterMap[ xRegisterMapIndex ].address;

    pxRegisterMap[ xRegisterMapIndex ].data[ xOffset ] = usREQUEST_REGISTER_VALUE;

    ucREPLY_FUNCTION_CODE     = ucREQUEST_FUNCTION_CODE;
    ucREPLY_ADDRESS_HI        = ucREQUEST_ADDRESS_HI;
    ucREPLY_ADDRESS_LO        = ucREQUEST_ADDRESS_LO;
    ucREPLY_REGISTER_VALUE_HI = ucREQUEST_REGISTER_VALUE_HI;
    ucREPLY_REGISTER_VALUE_LO = ucREQUEST_REGISTER_VALUE_LO;

    xReplyLength = 6;

    if( pxRegisterMap[ xRegisterMapIndex ].callback != NULL )
    {
        ( pxRegisterMap[ xRegisterMapIndex ].callback )();
    }
}
/*-----------------------------------------------------------*/

void SerialModbusServer::vHandlerFC08( void )
{
    /* Set the common reply data for all diagnostic sub functions. */
    ucREPLY_FUNCTION_CODE        = ucREQUEST_FUNCTION_CODE;
    ucREPLY_SUB_FUNCTION_CODE_HI = ucREQUEST_SUB_FUNCTION_CODE_HI;
    ucREPLY_SUB_FUNCTION_CODE_LO = ucREQUEST_SUB_FUNCTION_CODE_LO;

    /* Some of the diagnoostic sub functions just return the received
    request data (which is in most cases 0x0000). So we apply this data
    directly at the beginning of the handler and will change it only in the
    specific cases - the same goes for the reply length. */
    ucREPLY_DATA_HI = ucREQUEST_DATA_HI;
    ucREPLY_DATA_LO = ucREQUEST_DATA_LO;
    xReplyLength = 6;

    switch( usREQUEST_SUB_FUNCTION_CODE )
    {
#if( configSFC00 == 1 )
        case RETURN_QUERY_DATA:
        {
            xReplyLength = 4;

            for( ; xReplyLength < xRequestLength - xChecksumLength; xReplyLength++ )
            {
                pucReplyFrame[ xReplyLength ] = pucRequestFrame[ xReplyLength ];
            }

            break;
        }
#endif
#if( configSFC01 == 1 )
        case RESTART_COMMUNICATIONS_OPTION:
        {
            if( ( usREQUEST_DATA == 0x0000 ) || ( usREQUEST_DATA == CLEAR_COM_EVENT_LOG ) )
            {
                /* INFO: The Modbus spec prescribes here to set an specific
                error flag, but this flag is nowhere specified. */

                /* Reset the Only Listen Mode. */
                bListenOnlyMode = false;

                if( usREQUEST_DATA == CLEAR_COM_EVENT_LOG )
                {
                    /* INFO: The Modbus spec prescribes here to clear the
                    communication event log, which is not implemented. */
                }

                /* INFO: The Modbus spec prescribes here to perform a complete
                restart of the device, but we skip that. */
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC02 == 1 )
        case RETURN_DIAGNOSTIC_REGISTER:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usDiagnosticRegister );
                ucREPLY_DATA_LO =  lowByte( usDiagnosticRegister );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC03 == 1 )
        case CHANGE_ASCII_INPUT_DELIMITER:
        {
            if( ( isAscii( ( char ) ucREQUEST_INPUT_DELIMITER_HI ) == true ) &&
                ( ucREQUEST_INPUT_DELIMITER_LO == 0x00 ) )
            {
                cAsciiInputDelimiter = ( char ) ucREQUEST_INPUT_DELIMITER_HI;

                ucREPLY_INPUT_DELIMITER_HI = ( uint8_t ) cAsciiInputDelimiter;
                ucREPLY_INPUT_DELIMITER_LO = 0x00;
            }
            else
            {
                #if( configEXTENDED_EXCEPTION_CODES == 1 )
                {
                    ( void ) xSetException( SERVER_ILLEGAL_INPUT_DELIMITER );
                }
                #else
                {
                    ( void ) xSetException( ILLEGAL_DATA_VALUE );
                }
                #endif

                return;
            }

            break;
        }
#endif
#if( configSFC04 == 1 )
        case FORCE_LISTEN_ONLY_MODE:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                bListenOnlyMode = true;
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC10 == 1 )
        case CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                vClearDiagnosticCounters();
                diagRegClear();

                xReplyLength = xRequestLength;
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC11 == 1 )
        case RETURN_BUS_MESSAGE_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usBusMessageCount );
                ucREPLY_DATA_LO =  lowByte( usBusMessageCount );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC12 == 1 )
        case RETURN_BUS_COMMUNICATION_ERROR_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usBusCommunicationErrorCount );
                ucREPLY_DATA_LO =  lowByte( usBusCommunicationErrorCount );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC13 == 1 )
        case RETURN_BUS_EXCEPTION_ERROR_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerExceptionErrorCount );
                ucREPLY_DATA_LO =  lowByte( usServerExceptionErrorCount );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC14 == 1 )
        case RETURN_SERVER_MESSAGE_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerMessageCount );
                ucREPLY_DATA_LO =  lowByte( usServerMessageCount );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC15 == 1 )
        case RETURN_SERVER_NO_RESPONSE_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerNoResponseCount );
                ucREPLY_DATA_LO =  lowByte( usServerNoResponseCount );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC16 == 1 )
        case RETURN_SERVER_NAK_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerNAKCount );
                ucREPLY_DATA_LO =  lowByte( usServerNAKCount );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC17 == 1 )
        case RETURN_SERVER_BUSY_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usServerBusyCount );
                ucREPLY_DATA_LO =  lowByte( usServerBusyCount );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC18 == 1 )
        case RETURN_BUS_CHARACTER_OVERRUN_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usBusCharacterOverrunCount );
                ucREPLY_DATA_LO =  lowByte( usBusCharacterOverrunCount );
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC20 == 1 )
        case CLEAR_OVERRUN_COUNTER_AND_FLAG:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                usBusCharacterOverrunCount = 0;

                /* INFO: The Modbus spec prescribes here to also clear an error
                flag, but this flag is nowhere specified. */
            }
            else
            {
                ( void ) xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
        default:
        {
            #if( configEXTENDED_EXCEPTION_CODES == 1 )
            {
                ( void ) xSetException( SERVER_ILLEGAL_SUB_FUNCTION );
            }
            #else
            {
                ( void ) xSetException( ILLEGAL_FUNCTION );
            }
            #endif
        }
    }

    if( xException == OK )
    {
        if( pxRegisterMap[ xRegisterMapIndex ].callback != NULL )
        {
            ( pxRegisterMap[ xRegisterMapIndex ].callback )();
        }
    }
}
/*-----------------------------------------------------------*/

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
/*-----------------------------------------------------------*/

uint16_t SerialModbusServer::diagRegGet( void )
{
    return usDiagnosticRegister;
}
/*-----------------------------------------------------------*/

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
/*-----------------------------------------------------------*/

bool SerialModbusServer::diagRegSet( size_t bit )
{
    if( bit <= 15 )
    {
        bitSet( usDiagnosticRegister, bit );
        return true;
    }

    return false;
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::diagRegClear( size_t bit )
{
    if( bit <= 15 )
    {
        bitClear( usDiagnosticRegister, bit );
        return true;
    }

    return false;
}
/*-----------------------------------------------------------*/

void SerialModbusServer::diagRegClear( void )
{
    usDiagnosticRegister = 0x0000;
}
/*-----------------------------------------------------------*/

void SerialModbusServer::vHandlerFC16( void )
{
    size_t xOffset = 0;

    if( ( usREQUEST_QUANTITY >= 0x0001 ) && ( usREQUEST_QUANTITY <= 0x007B ) )
    {
        if( ucREQUEST_BYTE_COUNT_FC16 == ( ( uint8_t ) usREQUEST_QUANTITY * 2 ) )
        {
            xOffset = ( size_t ) ( usREQUEST_ADDRESS - pxRegisterMap[ xRegisterMapIndex ].address );

            if( ( ( size_t ) usREQUEST_QUANTITY + xOffset ) <= pxRegisterMap[ xRegisterMapIndex ].dataSize )
            {
                for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
                {
                    pxRegisterMap[ xRegisterMapIndex ].data[ i + xOffset ] = usRequestWord( i, 7 );
                }

                ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
                ucREPLY_ADDRESS_HI    = ucREQUEST_ADDRESS_HI;
                ucREPLY_ADDRESS_LO    = ucREQUEST_ADDRESS_LO;
                ucREPLY_QUANTITY_HI   = ucREQUEST_QUANTITY_HI;
                ucREPLY_QUANTITY_LO   = ucREQUEST_QUANTITY_LO;

                xReplyLength = 6;

                if( pxRegisterMap[ xRegisterMapIndex ].callback != NULL )
                {
                    ( pxRegisterMap[ xRegisterMapIndex ].callback )();
                }

                return;
            }

            ( void ) xSetException( ILLEGAL_DATA_ADDRESS );

            return;
        }
    }

    #if( configEXTENDED_EXCEPTION_CODES == 1 )
    {
        ( void ) xSetException( SERVER_ILLEGAL_QUANTITY );
    }
    #else
    {
        ( void ) xSetException( ILLEGAL_DATA_VALUE );
    }
    #endif
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::createRegister( MBAccess_t access, uint16_t address, size_t dataSize, uint8_t id )
{
    MBRegister_t * pxRegisterMapTmp = NULL;

    if( ( access == NA ) || ( dataSize == 0 ) || ( id == 0 ) || ( id > configID_SERVER_MAX ) )
    {
        return false;
    }

    if( pxRegisterMap == NULL )
    {
        pxRegisterMapTmp = ( MBRegister_t * ) malloc( sizeof( MBRegister_t ) * 2 );
        if( pxRegisterMapTmp != NULL )
        {
            ( void ) bClearRegisterMapEntry( &pxRegisterMapTmp[ 0 ] );
            xRegisterMapSize = 1;
        }
    }
    else
    {
        pxRegisterMapTmp = ( MBRegister_t * ) realloc( pxRegisterMap, sizeof( MBRegister_t ) * ( xRegisterMapSize + 1 ) );
    }

    if( pxRegisterMapTmp != NULL )
    {
        pxRegisterMapTmp[ xRegisterMapSize - 1 ].data = ( uint16_t * ) calloc( dataSize, sizeof( uint16_t ) );
        if( pxRegisterMapTmp[ xRegisterMapSize - 1 ].data != NULL )
        {
#if( configSERVER_MULTI_ID == 1 )
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].id       = id;
#endif
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].access   = access;
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].address  = address;
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].dataSize = dataSize;
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].callback = NULL;

            ( void ) bClearRegisterMapEntry( &pxRegisterMapTmp[ xRegisterMapSize ] );
            pxRegisterMap = pxRegisterMapTmp;
            xRegisterMapSize += 1;

            #if( configSERVER_MULTI_ID == 1 )
            {
                /* Just in case someone creates a register after calling the
                begin method, we re-initialize the id map. */
                vSetIdMap();
            }
            #endif

            return true;
        }
    }

    return false;
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::createCoils( uint16_t address, size_t dataSize, uint8_t id )
{
    return createRegister( RW, address, dataSize, id );
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::createInputResgisters( uint16_t address, size_t dataSize, uint8_t id )
{
    return createRegister( RD, address, dataSize, id );
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::createHoldingRegisters( uint16_t address, size_t dataSize, uint8_t id )
{
    return createRegister( RW, address, dataSize, id );
}
/*-----------------------------------------------------------*/

int32_t SerialModbusServer::lGetRegister( uint16_t address, uint8_t id )
{
    size_t xOffset = 0;

    if( ( id != 0 ) && ( id <= configID_SERVER_MAX ) )
    {
        for( size_t i = 0; pxRegisterMap[ i ].data != NULL; i++ )
        {
#if( configSERVER_MULTI_ID == 1 )
            if( id == pxRegisterMap[ i ].id )
            {
#endif
                if( ( address >= pxRegisterMap[ i ].address ) &&
                    ( address < ( pxRegisterMap[ i ].address + ( uint16_t ) pxRegisterMap[ i ].dataSize ) ) )
                {
                    xOffset = address - pxRegisterMap[ i ].address;
                    return ( int32_t ) pxRegisterMap[ i ].data[ xOffset ];
                }
#if( configSERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return ( int32_t ) -1;
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::bSetRegister( uint16_t address, uint16_t value, uint8_t id )
{
    size_t xOffset = 0;

    if( ( id != 0 ) && ( id <= configID_SERVER_MAX ) )
    {
        for( size_t i = 0; pxRegisterMap[ i ].data != NULL; i++ )
        {
#if( configSERVER_MULTI_ID == 1 )
            if( id == pxRegisterMap[ i ].id )
            {
#endif
                if( ( address >= pxRegisterMap[ i ].address ) &&
                    ( address < ( pxRegisterMap[ i ].address + ( uint16_t ) pxRegisterMap[ i ].dataSize ) ) )
                {
                    xOffset = address - pxRegisterMap[ i ].address;
                    pxRegisterMap[ i ].data[ xOffset ] = value;
                    return true;
                }
#if( configSERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return false;
}
/*-----------------------------------------------------------*/

int32_t SerialModbusServer::getCoil( uint16_t address, uint8_t id )
{
    return lGetRegister( address, id );
}
/*-----------------------------------------------------------*/

int32_t SerialModbusServer::getInputResgister( uint16_t address, uint8_t id )
{
    return lGetRegister( address, id );
}
/*-----------------------------------------------------------*/

int32_t SerialModbusServer::getHoldingRegister( uint16_t address, uint8_t id )
{
    return lGetRegister( address, id );
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::setCoil( uint16_t address, uint16_t value, uint8_t id )
{
    return bSetRegister( address, value, id );
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::setInputResgister( uint16_t address, uint16_t value, uint8_t id )
{
    return bSetRegister( address, value, id );
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::setHoldingRegister( uint16_t address, uint16_t value, uint8_t id )
{
    return bSetRegister( address, value, id );
}
/*-----------------------------------------------------------*/

bool SerialModbusServer::bClearRegisterMapEntry( MBRegister_t * pxRegisterMapEntry )
{
    if( pxRegisterMapEntry != NULL )
    {
#if( configSERVER_MULTI_ID == 1 )
        pxRegisterMapEntry->id       = 0xFF;
#endif
        pxRegisterMapEntry->access   = NA;
        pxRegisterMapEntry->address  = 0xFFFF;
        pxRegisterMapEntry->data     = NULL;
        pxRegisterMapEntry->dataSize = 0;
        pxRegisterMapEntry->callback = NULL;

        return true;
    }

    return false;
}
