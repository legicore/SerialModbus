////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusSlave.cpp
 *
 * @author      Martin Legleiter
 *
 * @brief       TODO
 * 
 * @copyright   2018 Martin Legleiter
 * 
 * @license     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <Arduino.h>

#include "SerialModbusConfig.h"
#include "SerialModbusBase.h"
#include "SerialModbusSlave.h"

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
        cpt1_busMsgCnt      = 0;
        cpt2_busComErrCnt   = 0;
        cpt3_slvExcErrCnt   = 0;
        cpt4_slvMsgCnt      = 0;
        cpt5_slvNoRspCnt    = 0;
        cpt6_slvNAKCnt      = 0;
        cpt7_slvBsyCnt      = 0;
        cpt8_busChrOvrCount = 0;
    }
    #endif

    bListenOnlyMode = false;

    pxRegisterMap = NULL;
    xRegisterMapIndex = 0;

    ucSlaveId = configID_SLAVE_MAX;
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::begin( uint8_t slaveId, uint32_t baud, HardwareSerial * serial, uint8_t config )
{
    ucSlaveId = slaveId;
    pxSerial = serial;
    pxSerial->begin( baud, config );
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::begin( uint8_t slaveId, uint32_t baud, SoftwareSerial * serial )
{
    ucSlaveId = slaveId;
    pxSerialSoftware = serial;
    pxSerialSoftware->begin( baud );
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::vSetState( MBSlaveState_t xStatePar )
{
    xState = xStatePar;
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::setRegisterMap( const MBRegister_t * registerMap )
{
    pxRegisterMap = registerMap;
}
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

                    if( bListenOnlyMode == false )
                    {
                        vSendData( pucReplyFrame, xReplyLength );
                    }
                    
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
                                if( ucREQUEST_ID == ucSlaveId )
                                {
                                    vStartInterFrameDelay();
                                    vStartInterCharacterTimeout();
                                    break;
                                }
                            }
                            #endif

                            #if( configMODE == configMODE_ASCII )
                            {
                                if( pucRequestFrame[ 0 ] != ( uint8_t ) ':' )
                                {
                                    vClearRequestFrame();
                                    break;
                                }
                            }
                            #endif

                            vClearRequestFrame();
                            break;
                        }
                    }
                    else
                    {
                        incCPT8();
                        xSetException( NOK_RX_OVERFLOW );
                        vSetState( FORMATTING_ERROR_REPLY );
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
                            if( xCheckChecksum( pucRequestFrame, xRequestLength ) == OK )
                            {
                                while( bTimeoutInterFrameDelay() != true );
                                vSetState( CHECKING_REQUEST );
                            }
                            else
                            {
                                vClearRequestFrame();
                                vSetState( SLAVE_IDLE );
                            }
                        }
                    }
                    #endif

                    #if( configMODE == configMODE_ASCII )
                    {
                        /* Check for Newline (frame end) */
                        if( pucRequestFrame[ xRequestLength - 1 ] == ( uint8_t ) cAsciiInputDelimiter )
                        {
                            /* Check for Carriage Return (frame end) */
                            if( pucRequestFrame[ xRequestLength - 2 ] == ( uint8_t ) '\r' )
                            {
                                /* Convert the frame from rtu to ascii format */
                                xAsciiToRtu( pucReplyFrame, &xReplyLength );
                                xAsciiToRtu( pucRequestFrame, &xRequestLength );

                                vSetState( CHECKING_REQUEST );
                            }
                        }
                    }
                    #endif
                }

                break;
            }

            case CHECKING_REQUEST :
            {
                incCPT1();

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
                incCPT4();

                switch( ucREQUEST_FUNCTION_CODE )
                {
#if( ( configFC03 == 1 ) || ( configFC04 == 1 ) )
                    case READ_HOLDING_REGISTERS :
                    case READ_INPUT_REGISTERS :
                    {
                        vHandler03_04();
                        break;
                    }
#endif
#if( configFC05 == 1 )
                    case WRITE_SINGLE_COIL :
                    {
                        vHandler05();
                        break;
                    }
#endif
#if( configFC06 == 1 )
                    case WRITE_SINGLE_REGISTER :
                    {
                        vHandler06();
                        break;
                    }
#endif
#if( configFC08 == 1 )
                    case DIAGNOSTIC :
                    {
                        vHandler08();
                        break;
                    }
#endif
#if( configFC16 == 1 )
                    case WRITE_MULTIPLE_REGISTERS :
                    {
                        vHandler16();
                        break;
                    }
#endif
                    default :
                    {
                        incCPT3();
                        xSetException( ILLEGAL_FUNCTION );
                    }
                }

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

            case FORMATTING_NORMAL_REPLY :
            {
                if( ucREQUEST_ID != configID_BROADCAST )
                {
                    ucREPLY_ID = ucSlaveId;
                    xSetChecksum( pucReplyFrame, &xReplyLength );
                }
                else
                {
                    incCPT5();
                    vClearReplyFrame();
                }

                vClearRequestFrame();
                xSetException( OK );
                vSetState( SLAVE_IDLE );

                break;
            }

            case FORMATTING_ERROR_REPLY :
            {
                if( ucREQUEST_ID != configID_BROADCAST )
                {
                    incCPT3();

                    ucREPLY_ID            = ucSlaveId;
                    ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE | 0x80;
                    ucREPLY_ERROR_CODE    = ( uint8_t ) xException;

                    xReplyLength = 3;

                    xSetChecksum( pucReplyFrame, &xReplyLength );
                }
                else
                {
                    vClearReplyFrame();
                }

                xSetException( OK );
                vClearRequestFrame();
                vSetState( SLAVE_IDLE );

                break;
            }

            default :
            {
                xSetException( NOK_PROCESS_STATE );
                vSetState( SLAVE_IDLE );
            }
        }

        #if( configPROCESS_LOOP_HOOK == 1 )
        {
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

    /* Reset the register map index. */
    xRegisterMapIndex = 0;

    /* Before we find a matching register map entry the exception will be set by
    default. If successful the exception is reset to 'OK' or will be overwritten
    if another error occurs. Otherwise it persists which means that we could not
    find a matching register map entry. */
    xSetException( ILLEGAL_DATA_ADDRESS );

    /* Scan the register map and check if the request address value lies in the
    range of one of the mapped register entries. */
    for( ; pxRegisterMap[ xRegisterMapIndex ].address != 0x0000; xRegisterMapIndex++ )
    {
        if( ( usReqAddress >= pxRegisterMap[ xRegisterMapIndex ].address ) &&
            ( usReqAddress < ( pxRegisterMap[ xRegisterMapIndex ].address + ( uint16_t ) pxRegisterMap[ xRegisterMapIndex ].objectSize ) ) )
        {
            /* Scan the access rights map for the request function code. */
            for( size_t i = 0; pxAccessRights[ i ].uxAccess != 0b00; i++ )
            {
                if( ucReqFunctionCode == ( uint8_t ) pxAccessRights[ i ].uxFunctionCode )
                {
                    /* Check if the type of the request function code has the
                    right to access the destination register. */
                    if( ( pxRegisterMap[ xRegisterMapIndex ].access & ( MBAccess_t ) pxAccessRights[ i ].uxAccess ) != 0 )
                    {
                        /* Reset the exception which was set from the start. */
                        xSetException( OK );
                        return OK;
                    }

                    /* While register access rights are not a standard feature
                    of Modbus we will send a error reply with a non standard
                    exception code. */
                    xSetException( NOK_ACCESS_RIGHT );
                    return NOK;
                }
            }

            /* We could not find the function code in the access rights map so
            we set the exception and abort the for loop. */
            xSetException( ILLEGAL_FUNCTION );
            break;
        }
    }

    incCPT3();

    return NOK;
}
/*-----------------------------------------------------------*/

#if( ( configFC03 == 1 ) || ( configFC04 == 1 ) )
void SerialModbusSlave::vHandler03_04( void )
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

    xSetException( ILLEGAL_DATA_VALUE );
}
#endif
/*-----------------------------------------------------------*/

#if( configFC05 == 1 )
void SerialModbusSlave::vHandler05( void )
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

    xSetException( ILLEGAL_DATA_VALUE );
}
#endif
/*-----------------------------------------------------------*/

#if( configFC06 == 1 )
void SerialModbusSlave::vHandler06( void )
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
#endif
/*-----------------------------------------------------------*/

#if( configFC08 == 1 )
void SerialModbusSlave::handler08( void )
{
    ucREPLY_FUNCTION_CODE        = ucREQUEST_FUNCTION_CODE;
    mbREPLY_SUB_FUNCTION_CODE_HI = mbREQUEST_SUB_FUNCTION_CODE_HI;
    mbREPLY_SUB_FUNCTION_CODE_LO = mbREQUEST_SUB_FUNCTION_CODE_LO;

    /* xxx */
    pucReplyFrame[ 4 ] = 0x00;
    pucReplyFrame[ 5 ] = 0x00;

    switch( mbREPLY_SUB_FUNCTION_CODE )
    {
        case RETURN_QUERY_DATA:
        {
            for( size_t i = 4; i < xRequestLength - 2; i++ )
            {
                pucReplyFrame[ i ] = pucRequestFrame[ i ];
            }

            xReplyLength = xRequestLength;

            break;
        }
        case RESTART_COMMUNICATIONS_OPTION:
        {
            // TODO
            break;
        }
        case RETURN_DIAGNOSTIC_REGISTER:
        {
            pucReplyFrame[ 4 ] = highByte( diagnosticRegister );
            pucReplyFrame[ 5 ] =  lowByte( diagnosticRegister );

            xReplyLength = xRequestLength;

            break;
        }
        case CHANGE_ASCII_INPUT_DELIMITER:
        {
            if( ( isAscii( mbREQUEST_INPUT_DELIMITER_HI ) == true ) &&
                ( mbREQUEST_INPUT_DELIMITER_LO            == 0x00 ) )
            {
                cAsciiInputDelimiter = ( char ) mbREQUEST_INPUT_DELIMITER_HI;
                mbREPLY_INPUT_DELIMITER_HI = ( uint8_t ) cAsciiInputDelimiter;
                mbREPLY_INPUT_DELIMITER_LO = mbREQUEST_INPUT_DELIMITER_LO;

                xReplyLength = xRequestLength;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case FORCE_LISTEN_ONLY_MODE:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                bListenOnlyMode = true;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                cpt1_busMsgCnt      = 0;
                cpt2_busComErrCnt   = 0;
                cpt3_slvExcErrCnt   = 0;
                cpt4_slvMsgCnt      = 0;
                cpt5_slvNoRspCnt    = 0;
                cpt6_slvNAKCnt      = 0;
                cpt7_slvBsyCnt      = 0;
                cpt8_busChrOvrCount = 0;

                diagRegClear();

                xReplyLength = xRequestLength;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case RETURN_BUS_MESSAGE_COUNT:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                mbREPLY_DATA_HI = highByte( cpt1_busMsgCnt );
                mbREPLY_DATA_LO =  lowByte( cpt1_busMsgCnt );

                xReplyLength = 7;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case RETURN_BUS_COMMUNICATION_ERROR_COUNT:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                mbREPLY_DATA_HI = highByte( cpt2_busComErrCnt );
                mbREPLY_DATA_LO =  lowByte( cpt2_busComErrCnt );

                xReplyLength = 7;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case RETURN_BUS_EXCEPTION_ERROR_COUNT:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                mbREPLY_DATA_HI = highByte( cpt3_slvExcErrCnt );
                mbREPLY_DATA_LO =  lowByte( cpt3_slvExcErrCnt );

                xReplyLength = 7;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case RETURN_SLAVE_MESSAGE_COUNT:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                mbREPLY_DATA_HI = highByte( cpt4_slvMsgCnt );
                mbREPLY_DATA_LO =  lowByte( cpt4_slvMsgCnt );

                xReplyLength = 7;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case RETURN_SLAVE_NO_RESPONSE_COUNT:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                mbREPLY_DATA_HI = highByte( cpt5_slvNoRspCnt );
                mbREPLY_DATA_LO =  lowByte( cpt5_slvNoRspCnt );

                xReplyLength = 7;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case RETURN_SLAVE_NAK_COUNT:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                mbREPLY_DATA_HI = highByte( cpt6_slvNAKCnt );
                mbREPLY_DATA_LO =  lowByte( cpt6_slvNAKCnt );

                xReplyLength = 7;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case RETURN_SLAVE_BUSY_COUNT:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                mbREPLY_DATA_HI = highByte( cpt7_slvBsyCnt );
                mbREPLY_DATA_LO =  lowByte( cpt7_slvBsyCnt );

                xReplyLength = 7;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case RETURN_BUS_CHARACTER_OVERRUN_COUNT:
        {
            if( mbREQUEST_DATA == 0x0000 )
            {
                mbREPLY_DATA_HI = highByte( cpt8_busChrOvrCount );
                mbREPLY_DATA_LO =  lowByte( cpt8_busChrOvrCount );

                xReplyLength = 7;
            }
            else
            {
                setException( ILLEGAL_DATA_VALUE );
            }

            break;
        }
        case CLEAR_OVERRUN_COUNTER_AND_FLAG:
        {
            // TODO
            break;
        }
        default:
        {
            incCPT3();
            setException( ILLEGAL_FUNCTION );
            return;
        }
    }

    if( mbDataList[ mbDataListIndex ].action != NULL )
    {
        (*mbDataList[ mbDataListIndex ].action)();
    }
}
/*-----------------------------------------------------------*/

uint16_t SerialModbusSlave::diagRegGet( void )
{
    return diagnosticRegister;
}
/*-----------------------------------------------------------*/

bool SerialModbusSlave::diagRegGet( size_t bit )
{
    if( bit <= 15 )
    {
        if( bitRead( diagnosticRegister, bit ) == 1 )
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
        bitSet( diagnosticRegister, bit );
        return true;
    }

    return false;
}
/*-----------------------------------------------------------*/

bool SerialModbusSlave::diagRegClear( size_t bit )
{
    if( bit <= 15 )
    {
        bitClear( diagnosticRegister, bit );
        return true;
    }

    return false;
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::diagRegClear( void )
{
    diagnosticRegister = 0x0000;
}
#endif
/*-----------------------------------------------------------*/

#if( configFC16 == 1 )
void SerialModbusSlave::vHandler16( void )
{
    size_t xOffset = 0;

    if( ( usREQUEST_QUANTITY >= 0x0001 ) && ( usREQUEST_QUANTITY <= 0x007B ) )
    {
        xOffset = ( size_t ) ( usREQUEST_ADDRESS - pxRegisterMap[ xRegisterMapIndex ].address );

        if( ( size_t ) ( usREQUEST_QUANTITY + xOffset ) <= pxRegisterMap[ xRegisterMapIndex ].objectSize )
        {
            if( ucREQUEST_BYTE_COUNT_2 == ( uint8_t ) ( 2 * usREQUEST_QUANTITY ) )
            {
                for( size_t i = 0; i < usREQUEST_QUANTITY; i++ )
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

    xSetException( ILLEGAL_DATA_VALUE );
}
#endif
