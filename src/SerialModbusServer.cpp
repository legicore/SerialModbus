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
#include <ctype.h>

#include "SerialModbusConfig.h"
#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"
#include "SerialModbusServer.h"

#include <Arduino.h>
#if defined( configMB_SERIAL_SW )
    #include <SoftwareSerial.h>
#endif

/*----------------------------------------------------------------------------*/

struct MB_AccessRights_s
{
    unsigned uxAccess       : 2;
    unsigned uxFunctionCode : 6;
};

typedef struct MB_AccessRights_s MB_AccessRights_t;

static const MB_AccessRights_t pxAccessRights[] = {

    /* Bit Data Access */
#if( configMB_FC01 == 1 )
    { MB_RD, FC_READ_COILS },
#endif
#if( configMB_FC02 == 1 )
    { MB_RD, FC_READ_DISCRETE_INPUTS },
#endif
#if( configMB_FC05 == 1 )
    { MB_WR, FC_WRITE_SINGLE_COIL },
#endif
#if( configMB_FC15 == 1 )
    { MB_WR, FC_WRITE_MULTIPLE_COILS },
#endif

    /* Word Data Access */
#if( configMB_FC03 == 1 )
    { MB_RD, FC_READ_HOLDING_REGISTERS },
#endif
#if( configMB_FC04 == 1 )
    { MB_RD, FC_READ_INPUT_REGISTERS },
#endif
#if( configMB_FC06 == 1 )
    { MB_WR, FC_WRITE_SINGLE_REGISTER },
#endif
#if( configMB_FC16 == 1 )
    { MB_WR, FC_WRITE_MULTIPLE_REGISTERS },
#endif
#if( configMB_FC22 == 1 )
    { MB_WR, FC_MASK_WRITE_REGISTER },
#endif
#if( configMB_FC23 == 1 )
    { MB_RW, FC_READ_WRITE_MULTIPLE_REGISTERS },
#endif
#if( configMB_FC24 == 1 )
    { MB_RD, FC_READ_FIFO_QUEUE },
#endif

    /* File Record Data Access */
#if( configMB_FC20 == 1 )
    { MB_RD, FC_READ_FILE_RECORD },
#endif
#if( configMB_FC21 == 1 )
    { MB_WR, FC_WRITE_FILE_RECORD },
#endif

    /* Diagnostics */
#if( configMB_FC07 == 1 )
    { MB_RD, FC_READ_EXCEPTION_STATUS },
#endif
#if( configMB_FC08 == 1 )
    { MB_RW, FC_DIAGNOSTIC },
#endif
#if( configMB_FC11 == 1 )
    { MB_RD, FC_GET_COM_EVENT_COUNTER },
#endif
#if( configMB_FC12 == 1 )
    { MB_RD, FC_GET_COM_EVENT_LOG },
#endif
#if( configMB_FC17 == 1 )
    { MB_RD, FC_REPORT_SERVER_ID },
#endif

    /* Marks the end of the list. */
    { MB_NA, 0b000000 }
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

    pxRegisterMap = NULL;
    xRegisterMapIndex = 0;

    ucServerId = configMB_ID_SERVER_MAX;

    #if( configMB_SERVER_MULTI_ID == 1 )
    {
        xIdCount = 0;
    }
    #endif

    xRegisterMapSize = 0;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::begin( uint8_t id, uint32_t baud, MB_Serial_t * serial, uint32_t config )
{
    if( ( id == 0 ) || ( id > configMB_ID_SERVER_MAX ) )
    {
        return false;
    }

    ucServerId = id;

    #if( configMB_SERVER_MULTI_ID == 1 )
    {
        vSetIdMap();
    }
    #endif

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

        #if( configMB_SERVER_MULTI_ID == 1 )
        {
            vSetIdMap();
        }
        #endif

        return SerialModbusBase::begin( baud, serial );
    }

#endif
/*----------------------------------------------------------------------------*/

void SerialModbusServer::vSetState( MB_ServerState_t xStatePar )
{
    xState = xStatePar;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setRegisterMap( MB_Register_t * registerMap )
{
    if( registerMap != NULL )
    {
        pxRegisterMap = registerMap;
        return true;
    }

    return false;
}
/*----------------------------------------------------------------------------*/

MB_Status_t SerialModbusServer::checkRegisterMap( void )
{
    size_t a = 0;
    size_t b = 0;

    if( pxRegisterMap != NULL )
    {
        /* Count the number of entries in the register map and make sure that
         * the data field is defined correctly. */
        while( pxRegisterMap[ a ].dataSize != 0 )
        {
            if( pxRegisterMap[ a ].data == NULL )
            {
                return MB_SERVER_REG_DATA;
            }

            if( pxRegisterMap[ a ].dataSize == 0 )
            {
                return MB_SERVER_REG_DATA_SIZE;
            }

            a++;
        }

        if( a == 1 )
        {
            /* If there is only one entry in the register map, there is no need
             * to perform a overlap check. */
            return MB_OK;
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
                         * ( b[ 0 ] > a[ n-1 ] ) NOR ( a[ 0 ] > b[ n-1 ] ) */
                        if( !( ( pxRegisterMap[ b ].address > ( pxRegisterMap[ a ].address + pxRegisterMap[ a ].dataSize - 1 ) ) ||
                               ( pxRegisterMap[ a ].address > ( pxRegisterMap[ b ].address + pxRegisterMap[ b ].dataSize - 1 ) ) ) )
                        {
                            return MB_SERVER_REG_OVERLAP;
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
            case SERVER_IDLE :
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

            case CHECKING_REQUEST :
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

            case PROCESSING_REQUIRED_ACTION :
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
                ucREPLY_ERROR_CODE    = ( uint8_t ) xStatus;

                xReplyLength = 3;

                ( void ) xSetChecksum( pucReplyFrame, &xReplyLength );

                vClearRequestFrame();
                vSetState( SERVER_IDLE );

                break;
            }

            default :
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
    /* Do nothing if the register map is not set. */
    if( pxRegisterMap == NULL )
    {
        return MB_NOK;
    }

    /* If the MB_DIAGNOSTIC functions are enabled we don't need to do the normal
     * request check. All diagnoctic functions are a part of the Modbus protocol
     * (and don't need any definition as a register etc.). */
    if( ucREQUEST_FUNCTION_CODE == ( uint8_t ) FC_DIAGNOSTIC )
    {
        return MB_OK;
    }

    /* Reset the register map index. */
    xRegisterMapIndex = 0;

    /* Before we find a matching register map entry the exception will be set by
     * default. If successful the exception is reset to 'MB_OK' or will be
     * overwritten if another error occurs. Otherwise it persists which means
     * that we could not find a matching register map entry. */
    ( void ) xSetException( MB_ILLEGAL_DATA_ADDRESS );

    /* Scan the register map and check if the request address value lies in the
     * range of one of the mapped register entries. */
    for( ; pxRegisterMap[ xRegisterMapIndex ].dataSize != 0; xRegisterMapIndex++ )
    {
#if( configMB_SERVER_MULTI_ID == 1 )
        if( pxRegisterMap[ xRegisterMapIndex ].id == ucServerId )
        {
#endif
            if( ( usReqAddress >= pxRegisterMap[ xRegisterMapIndex ].address ) &&
                ( usReqAddress < ( pxRegisterMap[ xRegisterMapIndex ].address + ( uint16_t ) pxRegisterMap[ xRegisterMapIndex ].dataSize ) ) )
            {
                /* Scan the access rights map for the request function code. */
                for( size_t i = 0; pxAccessRights[ i ].uxAccess != MB_NA; i++ )
                {
                    if( ucReqFunctionCode == ( uint8_t ) pxAccessRights[ i ].uxFunctionCode )
                    {
                        /* Check if the type of the request function code has
                         * the right to access the destination register. */
                        if( ( pxRegisterMap[ xRegisterMapIndex ].access & ( MB_Access_t ) pxAccessRights[ i ].uxAccess ) != 0 )
                        {
                            /* Reset the exception which was set from start. */
                            return xSetException( MB_OK );
                        }

                        /* While register access rights are not a standard
                         * feature of Modbus we will set a non standard
                         * exception and abort the for loop. */
                        #if( configMB_EXT_EXCEPTION_CODES == 1 )
                        {
                            ( void ) xSetException( MB_SERVER_ILLEGAL_ACCESS );
                        }
                        #else
                        {
                            ( void ) xSetException( MB_ILLEGAL_FUNCTION );
                        }
                        #endif

                        return MB_NOK;
                    }
                }

                /* We could not find the function code in the access rights map
                 * so we set the exception and abort the for loop. */
                #if( configMB_EXT_EXCEPTION_CODES == 1 )
                {
                    ( void ) xSetException( MB_SERVER_ILLEGAL_FUNCTION );
                }
                #else
                {
                    ( void ) xSetException( MB_ILLEGAL_FUNCTION );
                }
                #endif

                break;
            }
#if( configMB_SERVER_MULTI_ID == 1 )
        }
#endif
    }

    return MB_NOK;
}
/*----------------------------------------------------------------------------*/

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
    if( ( usREQUEST_COIL_VALUE == MB_COIL_ON ) || ( usREQUEST_COIL_VALUE == MB_COIL_OFF ) )
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
            if( ( isAscii( ( char ) ucREQUEST_INPUT_DELIMITER_HI ) == true ) &&
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
        if( pxRegisterMap[ xRegisterMapIndex ].callback != NULL )
        {
            ( pxRegisterMap[ xRegisterMapIndex ].callback )();
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

bool SerialModbusServer::createRegister( MB_Access_t access, uint16_t address, size_t dataSize, uint8_t id )
{
    MB_Register_t * pxRegisterMapTmp = NULL;

    if( ( access == MB_NA ) || ( dataSize == 0 ) || ( id == 0 ) || ( id > configMB_ID_SERVER_MAX ) )
    {
        return false;
    }

    if( pxRegisterMap == NULL )
    {
        pxRegisterMapTmp = ( MB_Register_t * ) malloc( sizeof( MB_Register_t ) * 2 );
        if( pxRegisterMapTmp != NULL )
        {
            ( void ) bClearRegisterMapEntry( &pxRegisterMapTmp[ 0 ] );
            xRegisterMapSize = 1;
        }
    }
    else
    {
        pxRegisterMapTmp = ( MB_Register_t * ) realloc( pxRegisterMap, sizeof( MB_Register_t ) * ( xRegisterMapSize + 1 ) );
    }

    if( pxRegisterMapTmp != NULL )
    {
        pxRegisterMapTmp[ xRegisterMapSize - 1 ].data = ( uint16_t * ) calloc( dataSize, sizeof( uint16_t ) );
        if( pxRegisterMapTmp[ xRegisterMapSize - 1 ].data != NULL )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].id       = id;
#endif
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].access   = access;
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].address  = address;
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].dataSize = dataSize;
            pxRegisterMapTmp[ xRegisterMapSize - 1 ].callback = NULL;

            ( void ) bClearRegisterMapEntry( &pxRegisterMapTmp[ xRegisterMapSize ] );
            pxRegisterMap = pxRegisterMapTmp;
            xRegisterMapSize += 1;

            #if( configMB_SERVER_MULTI_ID == 1 )
            {
                /* Just in case someone creates a register after calling the
                 * begin method, we re-initialize the id map. */
                vSetIdMap();
            }
            #endif

            return true;
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::createCoils( uint16_t address, size_t dataSize, uint8_t id )
{
    return createRegister( MB_RW, address, dataSize, id );
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::createInputResgisters( uint16_t address, size_t dataSize, uint8_t id )
{
    return createRegister( MB_RD, address, dataSize, id );
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::createHoldingRegisters( uint16_t address, size_t dataSize, uint8_t id )
{
    return createRegister( MB_RW, address, dataSize, id );
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusServer::lGetRegister( uint16_t address, uint8_t id )
{
    size_t xOffset = 0;

    if( ( id != 0 ) && ( id <= configMB_ID_SERVER_MAX ) )
    {
        for( size_t i = 0; pxRegisterMap[ i ].dataSize != 0; i++ )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
            if( id == pxRegisterMap[ i ].id )
            {
#endif
                if( ( address >= pxRegisterMap[ i ].address ) &&
                    ( address < ( pxRegisterMap[ i ].address + ( uint16_t ) pxRegisterMap[ i ].dataSize ) ) )
                {
                    xOffset = address - pxRegisterMap[ i ].address;
                    return ( int32_t ) pxRegisterMap[ i ].data[ xOffset ];
                }
#if( configMB_SERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return ( int32_t ) -1;
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bSetRegister( uint16_t address, uint16_t value, uint8_t id )
{
    size_t xOffset = 0;

    if( ( id != 0 ) && ( id <= configMB_ID_SERVER_MAX ) )
    {
        for( size_t i = 0; pxRegisterMap[ i ].dataSize != 0; i++ )
        {
#if( configMB_SERVER_MULTI_ID == 1 )
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
#if( configMB_SERVER_MULTI_ID == 1 )
            }
#endif
        }
    }

    return false;
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusServer::getCoil( uint16_t address, uint8_t id )
{
    return lGetRegister( address, id );
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusServer::getInputResgister( uint16_t address, uint8_t id )
{
    return lGetRegister( address, id );
}
/*----------------------------------------------------------------------------*/

int32_t SerialModbusServer::getHoldingRegister( uint16_t address, uint8_t id )
{
    return lGetRegister( address, id );
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setCoil( uint16_t address, uint16_t value, uint8_t id )
{
    return bSetRegister( address, value, id );
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setInputResgister( uint16_t address, uint16_t value, uint8_t id )
{
    return bSetRegister( address, value, id );
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::setHoldingRegister( uint16_t address, uint16_t value, uint8_t id )
{
    return bSetRegister( address, value, id );
}
/*----------------------------------------------------------------------------*/

bool SerialModbusServer::bClearRegisterMapEntry( MB_Register_t * pxRegisterMapEntry )
{
    if( pxRegisterMapEntry != NULL )
    {
#if( configMB_SERVER_MULTI_ID == 1 )
        pxRegisterMapEntry->id       = 0xFF;
#endif
        pxRegisterMapEntry->access   = MB_NA;
        pxRegisterMapEntry->address  = 0xFFFF;
        pxRegisterMapEntry->data     = NULL;
        pxRegisterMapEntry->dataSize = 0;
        pxRegisterMapEntry->callback = NULL;

        return true;
    }

    return false;
}
