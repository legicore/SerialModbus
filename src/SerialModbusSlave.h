///////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusSlave.h
 *
 * @author      legicore
 *
 * @brief       xxx
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_SLAVE_H__
#define __SERIAL_MODBUS_SLAVE_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusBase.h"

/*-----------------------------------------------------------*/

/**
 * @typedef MBSlaveState_t
 *          TODO
 */
typedef enum
{
    SLAVE_IDLE,
    CHECKING_REQUEST,
    PROCESSING_REQUIRED_ACTION,
    FORMATTING_NORMAL_REPLY,
    FORMATTING_ERROR_REPLY
}
MBSlaveState_t;

/*-----------------------------------------------------------*/

class SerialModbusSlave : public SerialModbusBase
{
public:

    SerialModbusSlave();
    MBStatus_t processModbus( void );

private:

    uint8_t ucSlaveId;
    bool bCheckSlaveId( uint8_t ucRequestId );
    MBSlaveState_t xState;
    void vSetState( MBSlaveState_t xStatePar );
    bool bScanDataList( void );
#if( configFC03 == 1 || configFC04 == 1 )
    void vHandler03_04( void );
#endif
#if( configFC05 == 1 )
    void vHandler05( void );
#endif
#if( configFC06 == 1 )
    void vHandler06( void );
#endif
#if( configFC16 == 1 )
    void vHandler16( void );
#endif
#if( configFC08 == 1 )
    void handler08( void );
    uint16_t cpt1_busMsgCnt;
    uint16_t cpt2_busComErrCnt;
    uint16_t cpt3_slvExcErrCnt;
    uint16_t cpt4_slvMsgCnt;
    uint16_t cpt5_slvNoRspCnt;
    uint16_t cpt6_slvNAKCnt;
    uint16_t cpt7_slvBsyCnt;
    uint16_t cpt8_busChrOvrCount;
    uint16_t diagnosticRegister;
    uint16_t diagRegGet( void );
    bool diagRegGet( size_t bit );
    bool diagRegSet( size_t bit );
    bool diagRegClear( size_t bit );
    void diagRegClear( void );
#endif
    bool bListenOnlyMode;
};
/*-----------------------------------------------------------*/

#if( configFC08 == 1 )
    #define incCPT( x ) if( x <= 0xFFFF ) { x++; }
    #define incCPT1()   incCPT( cpt1_busMsgCnt )
    #define incCPT2()   incCPT( cpt2_busComErrCnt )
    #define incCPT3()   incCPT( cpt3_slvExcErrCnt )
    #define incCPT4()   incCPT( cpt4_slvMsgCnt )
    #define incCPT5()   incCPT( cpt5_slvNoRspCnt )
    #define incCPT6()   incCPT( cpt6_slvNAKCnt )
    #define incCPT7()   incCPT( cpt7_slvBsyCnt )
    #define incCPT8()   incCPT( cpt8_busChrOvrCount )
#else
    #define incCPT1()
    #define incCPT2()
    #define incCPT3()
    #define incCPT4()
    #define incCPT5()
    #define incCPT6()
    #define incCPT7()
    #define incCPT8()
#endif

/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_SLAVE_H__ */
