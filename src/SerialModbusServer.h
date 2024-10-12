////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusServer.h
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

#ifndef __SERIAL_MODBUS_SERVER_H__
#define __SERIAL_MODBUS_SERVER_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"

#include <Arduino.h>
#if defined( configMB_SERIAL_SW )
    #include <SoftwareSerial.h>
#endif

/*-----------------------------------------------------------*/

enum MB_ServerState_e
{
    SERVER_IDLE,
    CHECKING_REQUEST,
    PROCESSING_REQUIRED_ACTION,
    FORMATTING_NORMAL_REPLY,
    FORMATTING_ERROR_REPLY
};

typedef enum MB_ServerState_e MB_ServerState_t;

enum MB_Access_e
{
    MB_NA = 0b00,   /* No Access */
    MB_RD = 0b01,   /* Read only */
    MB_WR = 0b10,   /* Write only */
    MB_RW = 0b11    /* Read and Write */
};

typedef enum MB_Access_e MB_Access_t;

struct MB_Register_s
{
#if( configMB_SERVER_MULTI_ID == 1 )
    uint8_t id;
#endif
    MB_Access_t access;
    uint16_t address;
    uint16_t * data;
    size_t dataSize;
    void (* callback)( void );
};

typedef struct MB_Register_s MB_Register_t;

#if( configMB_SERVER_MULTI_ID == 1 )
    #define MB_REGISTER_MAP_END { 0xFF, MB_NA, 0xFFFF, NULL, 0, NULL }
#else
    #define MB_REGISTER_MAP_END { MB_NA, 0xFFFF, NULL, 0, NULL }
#endif

/*-----------------------------------------------------------*/

class SerialModbusServer : public SerialModbusBase
{
public:

    SerialModbusServer();
    bool begin( uint8_t id, uint32_t baud, MB_Serial_t * serial = &SERIAL_PORT_HARDWARE, uint32_t config = configMB_SERIAL_CONF_DEFAULT );
#if defined( configMB_SERIAL_SW )
    bool begin( uint8_t id, uint32_t baud, MB_SWSerial_t * serial );
#endif
    MB_Status_t process( void );
    bool setRegisterMap( MB_Register_t * registerMap );
    MB_Status_t checkRegisterMap( void );

    /* Only for function code 8 (MB_DIAGNOSTIC). */

    uint16_t diagRegGet( void );
    bool diagRegGet( size_t bit );
    bool diagRegSet( size_t bit );
    bool diagRegClear( size_t bit );
    void diagRegClear( void );

    /* Simplified API functions. */

    bool createRegister( MB_Access_t access, uint16_t address, size_t dataSize, uint8_t id = configMB_ID_SERVER_MAX );
    bool createCoils( uint16_t address, size_t dataSize, uint8_t id = configMB_ID_SERVER_MAX );
    bool createInputResgisters( uint16_t address, size_t dataSize, uint8_t id = configMB_ID_SERVER_MAX );
    bool createHoldingRegisters( uint16_t address, size_t dataSize, uint8_t id = configMB_ID_SERVER_MAX );
    int32_t getCoil( uint16_t address, uint8_t id = configMB_ID_SERVER_MAX );
    int32_t getInputResgister( uint16_t address, uint8_t id = configMB_ID_SERVER_MAX );
    int32_t getHoldingRegister( uint16_t address, uint8_t id = configMB_ID_SERVER_MAX );
    bool setCoil( uint16_t address, uint16_t value, uint8_t id = configMB_ID_SERVER_MAX );
    bool setInputResgister( uint16_t address, uint16_t value, uint8_t id = configMB_ID_SERVER_MAX );
    bool setHoldingRegister( uint16_t address, uint16_t value, uint8_t id = configMB_ID_SERVER_MAX );

private:

    uint8_t ucServerId;
    MB_ServerState_t xState;
    void vSetState( MB_ServerState_t xStatePar );
    MB_Register_t * pxRegisterMap;
    size_t xRegisterMapIndex;
    MB_Status_t xCheckRequest( uint16_t usReqAddress, uint8_t ucReqFunctionCode );
    void vHandlerFC03_04( void );
    void vHandlerFC05( void );
    void vHandlerFC06( void );
    void vHandlerFC16( void );
    void vHandlerFC08( void );
    void vClearMB_DIAGNOSTICCounters( void );
    uint16_t usBusMessageCount;
    uint16_t usBusCommunicationErrorCount;
    uint16_t usServerExceptionErrorCount;
    uint16_t usServerMessageCount;
    uint16_t usServerNoResponseCount;
    uint16_t usServerNAKCount;
    uint16_t usServerBusyCount;
    uint16_t usBusCharacterOverrunCount;
    uint16_t usMB_DIAGNOSTICRegister;
    bool bListenOnlyMode;
#if( configMB_SERVER_MULTI_ID == 1 )
    uint8_t ucIdMap[ configMB_ID_COUNT_MAX ];
    size_t xIdCount;
    void vSetIdMap( void );
#endif
    bool bCheckId( uint8_t ucId );
    size_t xRegisterMapSize;
    int32_t lGetRegister( uint16_t address, uint8_t id = configMB_ID_SERVER_MAX );
    bool bSetRegister( uint16_t address, uint16_t value, uint8_t id = configMB_ID_SERVER_MAX );
    bool bClearRegisterMapEntry( MB_Register_t * pxRegisterMapEntry );
    bool bRegisterMapLock_sAPI;
    bool bRegisterMapLock;
};
/*-----------------------------------------------------------*/

#if( configMB_FC08 == 1 )

    #if( configMB_SFC11 == 1 )
        #define vIncCPT1()  usBusMessageCount++
    #else
        #define vIncCPT1()
    #endif
    #if( configMB_SFC12 == 1 )
        #define vIncCPT2()  usBusCommunicationErrorCount++
    #else
        #define vIncCPT2()
    #endif
    #if( configMB_SFC13 == 1 )
        #define vIncCPT3()  usServerExceptionErrorCount++
    #else
        #define vIncCPT3()
    #endif
    #if( configMB_SFC14 == 1 )
        #define vIncCPT4()  usServerMessageCount++
    #else
        #define vIncCPT4()
    #endif
    #if( configMB_SFC15 == 1 )
        #define vIncCPT5()  usServerNoResponseCount++
    #else
        #define vIncCPT5()
    #endif
    #if( configMB_SFC16 == 1 )
        #define vIncCPT6()  usServerNAKCount++
    #else
        #define vIncCPT6()
    #endif
    #if( configMB_SFC17 == 1 )
        #define vIncCPT7()  usServerBusyCount++
    #else
        #define vIncCPT7()
    #endif
    #if( configMB_SFC18 == 1 )
        #define vIncCPT8()  usBusCharacterOverrunCount++
    #else
        #define vIncCPT8()
    #endif

#else

    #define vIncCPT1()
    #define vIncCPT2()
    #define vIncCPT3()
    #define vIncCPT4()
    #define vIncCPT5()
    #define vIncCPT6()
    #define vIncCPT7()
    #define vIncCPT8()

#endif
/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_SERVER_H__ */
