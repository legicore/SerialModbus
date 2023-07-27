////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusClient.h
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

#ifndef __SERIAL_MODBUS_CLIENT_H__
#define __SERIAL_MODBUS_CLIENT_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusBase.h"

#include <Arduino.h>

/*-----------------------------------------------------------*/

enum MBClientState_e
{
    CLIENT_IDLE,
    WAITING_TURNAROUND_DELAY,
    WAITING_FOR_REPLY,
    PROCESSING_REPLY,
    PROCESSING_ERROR
};

typedef enum MBClientState_e MBClientState_t;

struct MBRequest_s
{
    uint8_t id;
    uint8_t functionCode;
    uint16_t address;
    uint16_t * data;
    size_t dataSize;
    void (*callback)( void );
};

typedef struct MBRequest_s MBRequest_t;

#define REQUEST_MAP_END { configID_SERVER_MAX, 0x00, 0x0000, NULL, 0, NULL }

/*-----------------------------------------------------------*/

class SerialModbusClient : public SerialModbusBase
{
public:

    SerialModbusClient();
    void setRequestMap( const MBRequest_t * requestMap );
    MBStatus_t setRequest( const MBRequest_t * request, bool requestMap = false );
    MBStatus_t processModbus( void );
    void setResponseTimeout( uint32_t timeMs );
    void setTurnaroundDelay( uint32_t timeMs );

    /* Simplified API functions. */

    int16_t sendRequest( uint8_t id, uint8_t functionCode, uint16_t address, uint16_t value = 0 );
    int16_t readHoldingRegister( uint8_t id, uint16_t address );
    int16_t readInputRegister( uint8_t id, uint16_t address );
    int16_t writeSingleCoil( uint8_t id, uint16_t address, uint16_t value );
    int16_t writeSingleRegister( uint8_t id, uint16_t address, uint16_t value );
    MBStatus_t getLastException( void );
    const char * getLastExceptionString( void );

private:

    MBClientState_t xState;
    void vSetState( MBClientState_t xStatePar );
    MBStatus_t xProcessRequestMap( void );
    const MBRequest_t * pxRequest;
    const MBRequest_t * pxRequestMap;
    size_t xRequestMapIndex;
    bool bSkipRequestMap;
    uint32_t ulTimerTurnaroundDelayUs;
    uint32_t ulTimerResponseTimeoutUs;
    uint32_t ulTurnaroundDelayUs;
    uint32_t ulResponseTimeoutUs;
    bool bTimeoutTurnaroundDelay( void );
    bool bTimeoutResponseTimeout( void );
    void vStartTurnaroundDelay( void );
    void vStartResponseTimeout( void );
    void vHandlerFC03_04( void );
    void vHandlerFC05( void );
    void vHandlerFC06( void );
    void vHandlerFC08( void );
    void vHandlerFC16( void );
    MBStatus_t xStatusSimpleAPI;
};
/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_CLIENT_H__ */
