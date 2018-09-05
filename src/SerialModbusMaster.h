///////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusMaster.h
 *
 * @author      legicore
 *
 * @brief       xxx
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_MASTER_H__
#define __SERIAL_MODBUS_MASTER_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusBase.h"

/*-----------------------------------------------------------*/

/**
 * @typedef MBMasterState_t
 *          TODO
 */
typedef enum MBMasterState_e
{
    MASTER_IDLE,
    WAITING_TURNAROUND_DELAY,
    WAITING_FOR_REPLY,
    PROCESSING_REPLY,
    PROCESSING_ERROR
}
MBMasterState_t;

/*-----------------------------------------------------------*/

class SerialModbusMaster : public SerialModbusBase
{
public:

    SerialModbusMaster();
    MBStatus_t setRequest( MBData_t * request );
    MBStatus_t processModbus( void );
    size_t getReplyDataSize( void );
    size_t getReplyData( uint16_t * buffer, size_t bufferSize );
    void setResponseTimeout( uint32_t timeMs );
    void setTurnaroundDelay( uint32_t timeMs );

private:

    MBMasterState_t xState;
    void vSetState( MBMasterState_t xStatePar );
    MBStatus_t xProcessDataList( void );
    MBData_t * pxRequestData;
    size_t xReplyDataSize;
    uint32_t ulTimerTurnaroundDelayUs;
    uint32_t ulTimerResponseTimeoutUs;
    uint32_t ulTurnaroundDelayUs;
    uint32_t ulResponseTimeoutUs;
    bool bTimeoutTurnaroundDelay( void );
    bool bTimeoutResponseTimeout( void );
    void vStartTurnaroundDelay( void );
    void vStartResponseTimeout( void );
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
};
/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_MASTER_H__ */
