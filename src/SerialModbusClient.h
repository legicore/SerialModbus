////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusClient.h
 * 
 * @author      Martin Legleiter
 * 
 * @brief       TODO
 * 
 * @copyright   (c) 2024 Martin Legleiter
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

enum MB_ClientState_e
{
    CLIENT_IDLE,
    WAITING_TURNAROUND_DELAY,
    WAITING_FOR_REPLY,
    PROCESSING_REPLY,
    PROCESSING_ERROR
};

typedef enum MB_ClientState_e MB_ClientState_t;

/*-----------------------------------------------------------*/

struct MB_Request_s
{
    uint8_t id;
    uint8_t functionCode;
    uint16_t address;
    uint16_t * data;
    size_t dataSize;
    void (* callback)( void );
};

typedef struct MB_Request_s MB_Request_t;

#define MB_REQUEST_MAP_END { 0xFF, 0x00, 0xFFFF, NULL, 0, NULL }

/*-----------------------------------------------------------*/

class SerialModbusClient : public SerialModbusBase
{
public:

    SerialModbusClient();
    void setRequestMap( const MB_Request_t * requestMap );
    MB_Status_t setRequest( const MB_Request_t * request, bool requestMap = false );
    MB_Status_t process( void );
    uint32_t getResponseTimeout( void );
    uint32_t getTurnaroundDelay( void );
    bool setResponseTimeout( uint32_t timeMs );
    bool setTurnaroundDelay( uint32_t timeMs );

    /* Simplified API functions. */

    int32_t sendRequest( uint8_t id, uint8_t functionCode, uint16_t address, uint16_t data = 0x0000 );
    int32_t readHoldingRegister( uint8_t id, uint16_t address );
    int32_t readInputRegister( uint8_t id, uint16_t address );
    int32_t writeSingleCoil( uint8_t id, uint16_t address, uint16_t value );
    int32_t writeSingleRegister( uint8_t id, uint16_t address, uint16_t value );
    MB_Status_t getLastException( void );
    const char * getLastExceptionString( void );

private:

    MB_ClientState_t xState;
    void vSetState( MB_ClientState_t xStatePar );
    MB_Status_t xProcessRequestMap( void );
    const MB_Request_t * pxRequest;
    const MB_Request_t * pxRequestMap;
    size_t xRequestMapIndex;
    bool bSkipRequestMap;
    uint32_t ulTimerTurnaroundDelayMs;
    uint32_t ulTimerResponseTimeoutMs;
    uint32_t ulTurnaroundDelayMs;
    uint32_t ulResponseTimeoutMs;
    bool bTimeoutTurnaroundDelay( void );
    bool bTimeoutResponseTimeout( void );
    void vStartTurnaroundDelay( void );
    void vStartResponseTimeout( void );
    void vHandlerFC03_04( void );
    void vHandlerFC05( void );
    void vHandlerFC06( void );
    void vHandlerFC08( void );
    void vHandlerFC16( void );
    MB_Status_t xStatusSimpleAPI;
};
/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_CLIENT_H__ */
