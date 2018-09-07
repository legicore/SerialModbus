///////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusSlave.cpp
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

#include "SerialModbusConfig.h"
#include "SerialModbusBase.h"
#include "SerialModbusSlave.h"

/*-----------------------------------------------------------*/

SerialModbusSlave::SerialModbusSlave()
{
    vSetState( SLAVE_IDLE );
    
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
}
/*-----------------------------------------------------------*/

void SerialModbusSlave::vSetState( MBSlaveState_t xStatePar )
{
    xState = xStatePar;
}
/*-----------------------------------------------------------*/

bool SerialModbusSlave::bCheckSlaveId( uint8_t ucRequestId )
{
    for( size_t i = 0; pxDataList[ i ].functionCode != 0x00; i++ )
    {
        if( ( ucRequestId == pxDataList[ i ].id ) || ( ucRequestId == configID_BROADCAST ) )
        {
            ucSlaveId = ucRequestId;
            return true;
        }
    }
            
    return false;
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
                                /* Start the inter frame delay */
                                vStartInterFrameDelay();
                                vStartInterCharacterTimeout();
                            }
                            #endif

                            #if( configMODE == configMODE_ASCII )
                            {
                                if( pucRequestFrame[ 0 ] != ':' )
                                {
                                    vClearRequestFrame();
                                    break;
                                }
                            }
                            #endif
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
                                xSetException( NOK_CHECKSUM );
                                vSetState( FORMATTING_ERROR_REPLY );
                            }
                        }
                    }
                    #endif

                    #if( configMODE == configMODE_ASCII )
                    {
                        /* Check for Newline (frame end) */
                        if( pucRequestFrame[ xRequestLength - 1 ] == cAsciiInputDelimiter )
                        {
                            /* Check for Carriage Return (frame end) */
                            if( pucRequestFrame[ xRequestLength - 2 ] == '\r' )
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

                if( bScanDataList() == true )
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
                if( ( bCheckSlaveId( ucREQUEST_ID ) == true ) || ( ucREQUEST_ID == configID_BROADCAST ) )
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
                
                xSetException( OK );
                vClearRequestFrame();
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

bool SerialModbusSlave::bScanDataList( void )
{
    bool bFunctionCode = false;
    
    for( size_t i = 0; pxDataList[ i ].functionCode != 0x00; i++ )
    {
        if( ( pxDataList[ i ].id == ucSlaveId ) || ( ucSlaveId == configID_BROADCAST ) )
        {
            if( pxDataList[ i ].functionCode == ucREQUEST_FUNCTION_CODE )
            {
                bFunctionCode = true;
                
                if( ( usREQUEST_ADDRESS >= pxDataList[ i ].address ) &&
                    ( usREQUEST_ADDRESS <  pxDataList[ i ].address + ( uint16_t ) pxDataList[ i ].objectSize ) )
                {
                    xDataListIndex = i;
                    return true;
                }
            }
        }
    }
    
    if( bFunctionCode == false )
    {
        return true;
    }

    incCPT3();
    xSetException( ILLEGAL_DATA_ADDRESS );
    
    return false;
}
/*-----------------------------------------------------------*/

#if( ( configFC03 == 1 ) || ( configFC04 == 1 ) )
void SerialModbusSlave::vHandler03_04( void )
{
    size_t xOffset = 0;

    if( ( usREQUEST_QUANTITY >= 0x0001 ) && ( usREQUEST_QUANTITY <= 0x007D ) )
    {
        xOffset = ( size_t ) usREQUEST_ADDRESS - pxDataList[ xDataListIndex ].address;
        
        if( ( ( size_t ) usREQUEST_QUANTITY + xOffset ) <= pxDataList[ xDataListIndex ].objectSize )
        {
            for( size_t i = 0; i < ( size_t ) usREQUEST_QUANTITY; i++ )
            {
                pucReplyFrame[ ( i * 2 ) + 3 ] = highByte( pxDataList[ xDataListIndex ].object[ i + xOffset ] );
                pucReplyFrame[ ( i * 2 ) + 4 ] =  lowByte( pxDataList[ xDataListIndex ].object[ i + xOffset ] );
            }
            
            ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
            ucREPLY_BYTE_COUNT    = ( uint8_t ) usREQUEST_QUANTITY * 2;
            
            xReplyLength = ( size_t ) ucREPLY_BYTE_COUNT + 3;

            if( pxDataList[ xDataListIndex ].action != NULL )
            {
                (*pxDataList[ xDataListIndex ].action)();
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
        pxDataList[ xDataListIndex ].object[ 0 ] = usREQUEST_COIL_VALUE;
        
        ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
        ucREPLY_ADDRESS_HI    = ucREQUEST_ADDRESS_HI;
        ucREPLY_ADDRESS_LO    = ucREQUEST_ADDRESS_LO;
        ucREPLY_COIL_VALUE_HI = ucREQUEST_COIL_VALUE_HI;
        ucREPLY_COIL_VALUE_LO = ucREQUEST_COIL_VALUE_LO;
        
        xReplyLength = 6;

        if( pxDataList[ xDataListIndex ].action != NULL )
        {
            (*pxDataList[ xDataListIndex ].action)();
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
	size_t xOffset = ( size_t ) usREQUEST_ADDRESS - pxDataList[ xDataListIndex ].address;
	
	pxDataList[ xDataListIndex ].object[ xOffset ] = usREQUEST_REGISTER_VALUE;
	
	ucREPLY_FUNCTION_CODE     = ucREQUEST_FUNCTION_CODE;
	ucREPLY_ADDRESS_HI        = ucREQUEST_ADDRESS_HI;
	ucREPLY_ADDRESS_LO        = ucREQUEST_ADDRESS_LO;
	ucREPLY_REGISTER_VALUE_HI = ucREQUEST_REGISTER_VALUE_HI;
	ucREPLY_REGISTER_VALUE_LO = ucREQUEST_REGISTER_VALUE_LO;
	
	xReplyLength = 6;

	if( pxDataList[ xDataListIndex ].action != NULL )
	{
		(*pxDataList[ xDataListIndex ].action)();
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
                asciicAsciiInputDelimiter = mbREQUEST_INPUT_DELIMITER_HI;
                mbREPLY_INPUT_DELIMITER_HI = asciicAsciiInputDelimiter;
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
        xOffset = ( size_t ) ( usREQUEST_ADDRESS - pxDataList[ xDataListIndex ].address );

        if( ( size_t ) ( usREQUEST_QUANTITY + xOffset ) <= pxDataList[ xDataListIndex ].objectSize )
        {
            if( ucREQUEST_BYTE_COUNT_2 == ( uint8_t ) ( 2 * usREQUEST_QUANTITY ) )
            {
                for( size_t i = 0; i < usREQUEST_QUANTITY; i++ )
                {
                    pxDataList[ xDataListIndex ].object[ i + xOffset ] = usRequestWord( i, 7 );
                }
                
                ucREPLY_FUNCTION_CODE = ucREQUEST_FUNCTION_CODE;
                ucREPLY_ADDRESS_HI    = ucREQUEST_ADDRESS_HI;
                ucREPLY_ADDRESS_LO    = ucREQUEST_ADDRESS_LO;
                ucREPLY_QUANTITY_HI   = ucREQUEST_QUANTITY_HI;
                ucREPLY_QUANTITY_LO   = ucREQUEST_QUANTITY_LO;
                
                xReplyLength = 6;

                if( pxDataList[ xDataListIndex ].action != NULL )
                {
                    (*pxDataList[ xDataListIndex ].action)();
                }
                
                return;
            }
        }
    }

    xSetException( ILLEGAL_DATA_VALUE );
}
#endif
