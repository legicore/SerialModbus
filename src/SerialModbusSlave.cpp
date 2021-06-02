////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusSlave.cpp
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
#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"
#include "SerialModbusSlave.h"

#include <Arduino.h>
#if defined( COMPAT_SOFTWARE_SERIAL )
    #include <SoftwareSerial.h>
#endif

/*-----------------------------------------------------------*/

typedef struct MBAccessRights_s
{
    unsigned uxAccess       : 2;
    unsigned uxFunctionCode : 6;
}
MBAccessRights_t;

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
    { RD, REPORT_SLAVE_ID },
#endif

    /* Marks the end of the list. */
    { 0b00, 0b000000 }
};
/*-----------------------------------------------------------*/

SerialModbusSlave::SerialModbusSlave()
{
    xState = SLAVE_IDLE;

    #if( configFC08 == 1 )
    {
        vClearDiagnosticCounters();
        usDiagnosticRegister = 0x0000;
    }
    #endif

    bListenOnlyMode = false;

    pxRegisterMap = NULL;
    xRegisterMapIndex = 0;

    ucSlaveId = 0xFF;

    #if( configSLAVE_MULTI_ID == 1 )
    {
        xIdCount = 0;
    }
    #endif
}
/*-----------------------------------------------------------*/

bool SerialModbusSlave::begin( uint8_t slaveId, uint32_t baud, Serial_t * serial, uint32_t config )
{
    if( slaveId == 0 || slaveId > configID_SLAVE_MAX )
    {
        return false;
    }

    ucSlaveId = slaveId;

    return SerialModbusBase::begin( baud, serial, config );
}
/*-----------------------------------------------------------*/

#if defined( COMPAT_SOFTWARE_SERIAL )

    bool SerialModbusSlave::begin( uint8_t slaveId, uint32_t baud, SoftwareSerial * serial )
    {
        if( slaveId == 0 || slaveId > configID_SLAVE_MAX )
        {
            return false;
        }

        ucSlaveId = slaveId;

        return SerialModbusBase::begin( baud, serial );
    }

#endif
/*-----------------------------------------------------------*/

void SerialModbusSlave::vSetState( MBSlaveState_t xStatePar )
{
    xState = xStatePar;
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::setRegisterMap( const MBRegister_t * registerMap )
{
    pxRegisterMap = registerMap;

    #if( configSLAVE_MULTI_ID == 1 )
    {
        vSetIdMap();
    }
    #endif
}
/*-----------------------------------------------------------*/

#if( configSLAVE_MULTI_ID == 1 )
    
    void SerialModbusSlave::vSetIdMap( void )
    {
        bool bIdFound = false;

        for( size_t i = 0; pxRegisterMap[ i ].id != 0x00; i++ )
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

#if( configSLAVE_MULTI_ID == 1 )
    
    bool SerialModbusSlave::bCheckId( uint8_t ucId )
    {
        for( size_t i = 0; i < xIdCount; i++ )
        {
            if( ucIdMap[ i ] == ucId )
            {
                return true;
            }
        }

        return false;
    }

#endif
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusSlave::processModbus( void )
{
    xSetException( OK );

    do
    {
        /* Get the current state and select the associated action */
        switch( xState )
        {
            case SLAVE_IDLE :
            {
                if( xReplyLength > 0 )
                {
                    #if( configMODE == configMODE_ASCII )
                    {
                        /* Convert request to ascii and update the pdu length */
                        xRtuToAscii( pucReplyFrame, &xReplyLength );
                    }
                    #endif

                    vSendData( pucReplyFrame, xReplyLength );
                    vClearReplyFrame();
                }
                else
                {
                    if( xRequestLength < configMAX_FRAME_SIZE )
                    {
                        if( bReceiveByte( pucRequestFrame, &xRequestLength ) == true )
                        {
                            #if( configMODE == configMODE_RTU )
                            {
#if( configSLAVE_MULTI_ID == 1 )
                                if( ( bCheckId( ucREQUEST_ID ) == true   ) ||
#else
                                if( ( ucREQUEST_ID == ucSlaveId          ) ||
#endif
                                    ( ucREQUEST_ID == configID_BROADCAST ) )
                                {
                                    vStartInterFrameDelay();
                                    vStartInterCharacterTimeout();
                                    break;
                                }
                            }
                            #endif

                            #if( configMODE == configMODE_ASCII )
                            {
                                if( pucRequestFrame[ 0 ] == ( uint8_t ) ':' )
                                {
                                    break;
                                }
                            }
                            #endif

                            vClearRequestFrame();
                        }
                    }
                    else
                    {
                        /* Receive buffer overflow -> Increment the bus
                        charakter overrun counter. */
                        vIncCPT8();
                        xSetException( CHARACTER_OVERRUN );

                        /* We go directly back to the idle state and don't send
                        any reply because it would cause bus collisions if every
                        slave sends an error reply. */
                        vClearRequestFrame();

                        vSetState( SLAVE_IDLE );
                        break;
                    }
                }

                /* Check if the start of a frame has been received. */
                if( xRequestLength > 0 )
                {
                    #if( configMODE == configMODE_RTU )
                    {
                        if( bTimeoutInterCharacterTimeout() == true )
                        {
                            if( xRequestLength > 3 )
                            {
                                if( xCheckChecksum( pucRequestFrame, xRequestLength ) == OK )
                                {
                                    /* Received a new valid request -> Increment
                                    the bus message counter. */
                                    vIncCPT1();

                                    while( bTimeoutInterFrameDelay() != true );

                                    #if( configSLAVE_MULTI_ID == 1 )
                                    {
                                        ucSlaveId = ucREQUEST_ID;
                                    }
                                    #endif

                                    vSetState( CHECKING_REQUEST );
                                    break;
                                }
                            }

                            /* Got an checksum error or received a frame that
                            consists of <= 3 characters -> Increment the bus
                            communication error counter. */
                            vIncCPT2();

                            /* We go directly back to the idle state and don't
                            send any reply because it would cause bus collisions
                            if every slave sends an error reply. */
                            vClearRequestFrame();
                            vSetState( SLAVE_IDLE );
                        }
                    }
                    #endif

                    #if( configMODE == configMODE_ASCII )
                    {
                        /* Check for the end of the frame which is in the
                        default configuration marked by a carriage return (0x3A)
                        followed by a line feed (0x0A - which is variable). */
                        if( pucRequestFrame[ xRequestLength - 1 ] == ( uint8_t ) cAsciiInputDelimiter )
                        {
                            if( pucRequestFrame[ xRequestLength - 2 ] == ( uint8_t ) '\r' )
                            {
                                /* Convert the request frame from ASCII to RTU
                                format and update the request length. */
                                xAsciiToRtu( pucRequestFrame, &xRequestLength );

                                /* Check if the received request is dedicated to
                                us or if it is a broadcast. */
#if( configSLAVE_MULTI_ID == 1 )
                                if( ( bCheckId( ucREQUEST_ID ) == true   ) ||
#else
                                if( ( ucREQUEST_ID == ucSlaveId          ) ||
#endif
                                    ( ucREQUEST_ID == configID_BROADCAST ) )
                                {
                                    if( xCheckChecksum( pucRequestFrame, xRequestLength ) == OK )
                                    {
                                        /* Received a new valid request ->
                                        Increment the bus message counter. */
                                        vIncCPT1();

                                        #if( configSLAVE_MULTI_ID == 1 )
                                        {
                                            ucSlaveId = ucREQUEST_ID;
                                        }
                                        #endif

                                        vSetState( CHECKING_REQUEST );
                                        break;
                                    }

                                    /* Checksum error -> Increment the bus
                                    communication error counter. */
                                    vIncCPT2();
                                }

                                /* We go directly back to the idle state because
                                we don't want to send any kind of reply. It
                                would cause bus collisions if every slave
                                sends an error reply. */
                                vClearRequestFrame();
                                vSetState( SLAVE_IDLE );
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
                addressed to the Id of this deveice -> Increment the slave
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
                /* If the Listen Only Mode is active we only monitor all bus
                messages, but we perform no data processing. Only a request with
                function code 8 (DIAGNOSTIC) and sub function code 1
                (RESTART_COMMUNICATIONS_OPTION) will be processed, because it is
                needed to deavtivate the only listen mode. */
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
                                xSetException( SLV_ILLEGAL_FUNCTION );
                            }
                            #else
                            {
                                xSetException( ILLEGAL_FUNCTION );
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
                vSetState( SLAVE_IDLE );

                break;
            }

            case FORMATTING_NORMAL_REPLY :
            {
                ucREPLY_ID = ucSlaveId;
                xSetChecksum( pucReplyFrame, &xReplyLength );

                vClearRequestFrame();
                vSetState( SLAVE_IDLE );

                break;
            }

            case FORMATTING_ERROR_REPLY :
            {
                vIncCPT3();

                ucREPLY_ID = ucSlaveId;
                ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE | 0x80;
                ucREPLY_ERROR_CODE = ( uint8_t ) xException;

                xReplyLength = 3;

                xSetChecksum( pucReplyFrame, &xReplyLength );

                vClearRequestFrame();
                vSetState( SLAVE_IDLE );

                break;
            }

            default :
            {
                #if( configEXTENDED_EXCEPTION_CODES == 1 )
                {
                    xSetException( SLV_ILLEGAL_STATE );
                }
                #else
                {
                    xSetException( SLAVE_DEVICE_FAILURE );
                }
                #endif

                vSetState( FORMATTING_ERROR_REPLY );
            }
        }

        #if( configPROCESS_LOOP_HOOK == 1 )
        {
            /* The process loop hook will only be executed when the state
            mashine is not in the idle state. Otherwise the loop hook would be
            execetued with every run through processModbus(). */
            if( ( vProcessLoopHook != NULL ) && ( xState != SLAVE_IDLE ) )
            {
                (*vProcessLoopHook)();
            }
        }
        #endif
    }
    while( xState != SLAVE_IDLE );

    return xException;
}
/*-----------------------------------------------------------*/

MBStatus_t SerialModbusSlave::xCheckRequest( uint16_t usReqAddress, uint8_t ucReqFunctionCode )
{
    /* Do nothing if the register map is not set. */
    if( pxRegisterMap == NULL )
    {
        return NOK;
    }

    #if( configFC08 == 1 )
    {
        /* If the diagnostic functions are enabled we don't need to do the
        normal request check. All diagnoctic functions are a part of the Modbus
        protocol (and don't need any definition as a register etc.). */
        if( ucREQUEST_FUNCTION_CODE == ( uint8_t ) DIAGNOSTIC )
        {
            return OK;
        }
    }
    #endif

    /* Reset the register map index. */
    xRegisterMapIndex = 0;

    /* Before we find a matching register map entry the exception will be set by
    default. If successful the exception is reset to 'OK' or will be overwritten
    if another error occurs. Otherwise it persists which means that we could not
    find a matching register map entry. */
    #if( configEXTENDED_EXCEPTION_CODES == 1 )
    {
        xSetException( SLV_ILLEGAL_DATA_ADDRESS );
    }
    #else
    {
        xSetException( ILLEGAL_DATA_ADDRESS );
    }
    #endif

    /* Scan the register map and check if the request address value lies in the
    range of one of the mapped register entries. */
    for( ; pxRegisterMap[ xRegisterMapIndex ].address != 0x0000; xRegisterMapIndex++ )
    {
#if( configSLAVE_MULTI_ID == 1 )
        if( pxRegisterMap[ xRegisterMapIndex ].id == ucSlaveId )
        {
#endif
            if( ( usReqAddress >= pxRegisterMap[ xRegisterMapIndex ].address ) &&
                ( usReqAddress < ( pxRegisterMap[ xRegisterMapIndex ].address + ( uint16_t ) pxRegisterMap[ xRegisterMapIndex ].objectSize ) ) )
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
                            xSetException( SLV_ILLEGAL_ACCESS );
                        }
                        #else
                        {
                            xSetException( SLAVE_DEVICE_FAILURE );
                        }
                        #endif

                        break;
                    }
                }

                /* We could not find the function code in the access rights map
                so we set the exception and abort the for loop. */
                #if( configEXTENDED_EXCEPTION_CODES == 1 )
                {
                    xSetException( SLV_ILLEGAL_FUNCTION );
                }
                #else
                {
                    xSetException( ILLEGAL_FUNCTION );
                }
                #endif

                break;
            }
#if( configSLAVE_MULTI_ID == 1 )
        }
#endif
    }

    return NOK;
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::vHandlerFC03_04( void )
{
    size_t xOffset = 0;

    if( ( usREQUEST_QUANTITY >= 0x0001 ) && ( usREQUEST_QUANTITY <= 0x007D ) )
    {
        xOffset = ( size_t ) usREQUEST_ADDRESS - pxRegisterMap[ xRegisterMapIndex ].address;

        if( ( ( size_t ) usREQUEST_QUANTITY + xOffset ) <= pxRegisterMap[ xRegisterMapIndex ].objectSize )
        {
            for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
            {
                pucReplyFrame[ ( i * 2 ) + 3 ] = highByte( pxRegisterMap[ xRegisterMapIndex ].object[ i + xOffset ] );
                pucReplyFrame[ ( i * 2 ) + 4 ] =  lowByte( pxRegisterMap[ xRegisterMapIndex ].object[ i + xOffset ] );
            }

            ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
            ucREPLY_BYTE_COUNT    = ( uint8_t ) usREQUEST_QUANTITY * 2;

            xReplyLength = ( size_t ) ucREPLY_BYTE_COUNT + 3;

            if( pxRegisterMap[ xRegisterMapIndex ].action != NULL )
            {
                (*pxRegisterMap[ xRegisterMapIndex ].action)();
            }

            return;
        }
    }

    #if( configEXTENDED_EXCEPTION_CODES == 1 )
    {
        xSetException( SLV_ILLEGAL_QUANTITY );
    }
    #else
    {
        xSetException( ILLEGAL_DATA_VALUE );
    }
    #endif
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::vHandlerFC05( void )
{
    if( ( usREQUEST_COIL_VALUE == COIL_ON ) || ( usREQUEST_COIL_VALUE == COIL_OFF ) )
    {
        pxRegisterMap[ xRegisterMapIndex ].object[ 0 ] = usREQUEST_COIL_VALUE;

        ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
        ucREPLY_ADDRESS_HI    = ucREQUEST_ADDRESS_HI;
        ucREPLY_ADDRESS_LO    = ucREQUEST_ADDRESS_LO;
        ucREPLY_COIL_VALUE_HI = ucREQUEST_COIL_VALUE_HI;
        ucREPLY_COIL_VALUE_LO = ucREQUEST_COIL_VALUE_LO;

        xReplyLength = 6;

        if( pxRegisterMap[ xRegisterMapIndex ].action != NULL )
        {
            (*pxRegisterMap[ xRegisterMapIndex ].action)();
        }

        return;
    }

    #if( configEXTENDED_EXCEPTION_CODES == 1 )
    {
        xSetException( SLV_ILLEGAL_COIL_VALUE );
    }
    #else
    {
        xSetException( ILLEGAL_DATA_VALUE );
    }
    #endif
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::vHandlerFC06( void )
{
    size_t xOffset = ( size_t ) usREQUEST_ADDRESS - pxRegisterMap[ xRegisterMapIndex ].address;

    pxRegisterMap[ xRegisterMapIndex ].object[ xOffset ] = usREQUEST_REGISTER_VALUE;

    ucREPLY_FUNCTION_CODE     = ucREQUEST_FUNCTION_CODE;
    ucREPLY_ADDRESS_HI        = ucREQUEST_ADDRESS_HI;
    ucREPLY_ADDRESS_LO        = ucREQUEST_ADDRESS_LO;
    ucREPLY_REGISTER_VALUE_HI = ucREQUEST_REGISTER_VALUE_HI;
    ucREPLY_REGISTER_VALUE_LO = ucREQUEST_REGISTER_VALUE_LO;

    xReplyLength = 6;

    if( pxRegisterMap[ xRegisterMapIndex ].action != NULL )
    {
        (*pxRegisterMap[ xRegisterMapIndex ].action)();
    }
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::vHandlerFC08( void )
{
    /* Set the common reply data for all diagnostic sub functions. */
    ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
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
                /* INFO: The Modbus spec prescribes here to also an specific
                error flag, but this flag is nowhere specified. */

                /* Reset the Only Listen Mode. */
                bListenOnlyMode = false;

                if( usREQUEST_DATA == CLEAR_COM_EVENT_LOG )
                {
                    /* INFO: The Modbus spec prescribes here to clear the
                    communication event log, which is not implemented. */
                }

                /* INFO: The Modbus spec prescribes here to perform a
                complete restart of the device, but we skip that. */
            }
            else
            {
                xSetException( ILLEGAL_DATA_VALUE );
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
                xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC03 == 1 )
        case CHANGE_ASCII_INPUT_DELIMITER:
        {
            if( ( isAscii( ( char ) ucREQUEST_INPUT_DELIMITER_HI ) == true ) &&
                ( ucREQUEST_INPUT_DELIMITER_LO                     == 0x00 ) )
            {
                cAsciiInputDelimiter = ( char ) ucREQUEST_INPUT_DELIMITER_HI;

                ucREPLY_INPUT_DELIMITER_HI = ( uint8_t ) cAsciiInputDelimiter;
                ucREPLY_INPUT_DELIMITER_LO = 0x00;
            }
            else
            {
                #if( configEXTENDED_EXCEPTION_CODES == 1 )
                {
                    xSetException( SLV_ILLEGAL_INPUT_DELIMITER );
                }
                #else
                {
                    xSetException( ILLEGAL_DATA_VALUE );
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
                xSetException( ILLEGAL_DATA_VALUE );
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
                xSetException( ILLEGAL_DATA_VALUE );
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
                xSetException( ILLEGAL_DATA_VALUE );
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
                xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC13 == 1 )
        case RETURN_BUS_EXCEPTION_ERROR_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usSlaveExceptionErrorCount );
                ucREPLY_DATA_LO =  lowByte( usSlaveExceptionErrorCount );
            }
            else
            {
                xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC14 == 1 )
        case RETURN_SLAVE_MESSAGE_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usSlaveMessageCount );
                ucREPLY_DATA_LO =  lowByte( usSlaveMessageCount );
            }
            else
            {
                xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC15 == 1 )
        case RETURN_SLAVE_NO_RESPONSE_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usSlaveNoResponseCount );
                ucREPLY_DATA_LO =  lowByte( usSlaveNoResponseCount );
            }
            else
            {
                xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC16 == 1 )
        case RETURN_SLAVE_NAK_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usSlaveNAKCount );
                ucREPLY_DATA_LO =  lowByte( usSlaveNAKCount );
            }
            else
            {
                xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
#if( configSFC17 == 1 )
        case RETURN_SLAVE_BUSY_COUNT:
        {
            if( usREQUEST_DATA == 0x0000 )
            {
                ucREPLY_DATA_HI = highByte( usSlaveBusyCount );
                ucREPLY_DATA_LO =  lowByte( usSlaveBusyCount );
            }
            else
            {
                xSetException( ILLEGAL_DATA_VALUE );
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
                xSetException( ILLEGAL_DATA_VALUE );
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

                /* INFO: The Modbus spec prescribes here to also clear an
                error flag, but this flag is nowhere specified. */
            }
            else
            {
                xSetException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
#endif
        default:
        {
            #if( configEXTENDED_EXCEPTION_CODES == 1 )
            {
                xSetException( SLV_ILLEGAL_SUB_FUNCTION );
            }
            #else
            {
                xSetException( ILLEGAL_FUNCTION );
            }
            #endif

            return;
        }
    }

    #if( configEXTENDED_EXCEPTION_CODES == 1 )
    {
        if( xException == ILLEGAL_DATA_VALUE )
        {
            xSetException( SLV_ILLEGAL_DATA_VALUE );
        }
    }
    #endif
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::vClearDiagnosticCounters( void )
{
    usBusMessageCount            = 0;
    usBusCommunicationErrorCount = 0;
    usSlaveExceptionErrorCount   = 0;
    usSlaveMessageCount          = 0;
    usSlaveNoResponseCount       = 0;
    usSlaveNAKCount              = 0;
    usSlaveBusyCount             = 0;
    usBusCharacterOverrunCount   = 0;
}
/*-----------------------------------------------------------*/

uint16_t SerialModbusSlave::diagRegGet( void )
{
    return usDiagnosticRegister;
}
/*-----------------------------------------------------------*/

bool SerialModbusSlave::diagRegGet( size_t bit )
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

bool SerialModbusSlave::diagRegSet( size_t bit )
{
    if( bit <= 15 )
    {
        bitSet( usDiagnosticRegister, bit );
        return true;
    }

    return false;
}
/*-----------------------------------------------------------*/

bool SerialModbusSlave::diagRegClear( size_t bit )
{
    if( bit <= 15 )
    {
        bitClear( usDiagnosticRegister, bit );
        return true;
    }

    return false;
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::diagRegClear( void )
{
    usDiagnosticRegister = 0x0000;
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::vHandlerFC16( void )
{
    size_t xOffset = 0;

    if( ( usREQUEST_QUANTITY >= 0x0001 ) && ( usREQUEST_QUANTITY <= 0x007B ) )
    {
        xOffset = ( size_t ) ( usREQUEST_ADDRESS - pxRegisterMap[ xRegisterMapIndex ].address );

        if( ( size_t ) usREQUEST_QUANTITY + xOffset <= pxRegisterMap[ xRegisterMapIndex ].objectSize )
        {
            if( ucREQUEST_BYTE_COUNT_2 == ( uint8_t ) ( 2 * usREQUEST_QUANTITY ) )
            {
                for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
                {
                    pxRegisterMap[ xRegisterMapIndex ].object[ i + xOffset ] = usRequestWord( i, 7 );
                }

                ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
                ucREPLY_ADDRESS_HI    = ucREQUEST_ADDRESS_HI;
                ucREPLY_ADDRESS_LO    = ucREQUEST_ADDRESS_LO;
                ucREPLY_QUANTITY_HI   = ucREQUEST_QUANTITY_HI;
                ucREPLY_QUANTITY_LO   = ucREQUEST_QUANTITY_LO;

                xReplyLength = 6;

                if( pxRegisterMap[ xRegisterMapIndex ].action != NULL )
                {
                    (*pxRegisterMap[ xRegisterMapIndex ].action)();
                }

                return;
            }
        }
    }

    #if( configEXTENDED_EXCEPTION_CODES == 1 )
    {
        xSetException( SLV_ILLEGAL_QUANTITY );
    }
    #else
    {
        xSetException( ILLEGAL_DATA_VALUE );
    }
    #endif
}
