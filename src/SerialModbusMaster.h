////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusMaster.h
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

#ifndef __SERIAL_MODBUS_MASTER_H__
#define __SERIAL_MODBUS_MASTER_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusConfig.h"
#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"

#include <Arduino.h>

/*-----------------------------------------------------------*/

/** TODO */
typedef enum MBMasterState_e
{
    MASTER_IDLE,
    WAITING_TURNAROUND_DELAY,
    WAITING_FOR_REPLY,
    PROCESSING_REPLY,
    PROCESSING_ERROR
}
MBMasterState_t;

/** TODO */
typedef struct MBRequest_s
{
    uint8_t id;
    uint8_t functionCode;
    uint16_t address;
    void * object;
    size_t objectSize;
    void (*action)( void );
}
MBRequest_t;

/** TODO */
#define REQUEST_MAP_END { configID_SLAVE_MAX, 0x00, 0x0000, NULL, 0, NULL }

/*-----------------------------------------------------------*/

/** TODO */
class SerialModbusMaster : public SerialModbusBase
{
public:

    SerialModbusMaster();
    void setRequestMap( const MBRequest_t * requestMap );
    MBStatus_t setRequest( const MBRequest_t * request, bool requestMap = false );
    MBStatus_t processModbus( void );
    void setResponseTimeout( uint32_t timeMs );
    void setTurnaroundDelay( uint32_t timeMs );

private:

    MBMasterState_t xState;
    void vSetState( MBMasterState_t xStatePar );
    MBStatus_t xProcessRequestMap( void );
    const MBRequest_t * pxRequest;
    const MBRequest_t * pxRequestMap;
    size_t xRequestMapIndex;
    bool bSkipRequestMap;
    uint32_t ulTimerTurnaroundDelayUs;
    uint32_t ulTimerResponseTimeoutUs;
    uint32_t ulTurnaroundDelayUs;
    uint32_t ulResponseTimeoutUs;
    bool bTimeoutTurnaroundDelay( void ) const;
    bool bTimeoutResponseTimeout( void ) const;
    void vStartTurnaroundDelay( void );
    void vStartResponseTimeout( void );
#if( configFC03 == 1 || configFC04 == 1 )
    void vHandlerFC03_04( void );
#endif
#if( configFC05 == 1 )
    void vHandlerFC05( void );
#endif
#if( configFC06 == 1 )
    void vHandlerFC06( void );
#endif
#if( configFC08 == 1 )
    void vHandlerFC08( void );
#endif
#if( configFC16 == 1 )
    void vHandlerFC16( void );
#endif
};
/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_MASTER_H__ */
