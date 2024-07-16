////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusServer.h
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

#ifndef __SERIAL_MODBUS_SERVER_H__
#define __SERIAL_MODBUS_SERVER_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"

#include <Arduino.h>
#if defined( COMPAT_SOFTWARE_SERIAL )
    #include <SoftwareSerial.h>
#endif

/*-----------------------------------------------------------*/

enum MBServerState_e
{
    SERVER_IDLE,
    CHECKING_REQUEST,
    PROCESSING_REQUIRED_ACTION,
    FORMATTING_NORMAL_REPLY,
    FORMATTING_ERROR_REPLY
};

typedef enum MBServerState_e MBServerState_t;

enum MBAccess_e
{
    NA = 0b00,  /** No Access */
    RD = 0b01,  /** Read only */
    WR = 0b10,  /** Write only */
    RW = 0b11   /** Read and write */
};

typedef enum MBAccess_e MBAccess_t;

struct MBRegister_s
{
#if( configSERVER_MULTI_ID == 1 )
    uint8_t id;
#endif
    MBAccess_t access;
    uint16_t address;
    uint16_t * data;
    size_t dataSize;
    void (* callback)( void );
};

typedef struct MBRegister_s MBRegister_t;

#if( configSERVER_MULTI_ID == 1 )
    #define REGISTER_MAP_END { 0xFF, NA, 0xFFFF, NULL, 0, NULL }
#else
    #define REGISTER_MAP_END { NA, 0xFFFF, NULL, 0, NULL }
#endif

/*-----------------------------------------------------------*/

class SerialModbusServer : public SerialModbusBase
{
public:

    SerialModbusServer();
    bool begin( uint8_t serverId, uint32_t baud, Serial_t * serial = &SERIAL_PORT_HARDWARE, uint32_t config = configSERIAL_CONF_DEFAULT );
#if defined( COMPAT_SOFTWARE_SERIAL )
    bool begin( uint8_t serverId, uint32_t baud, SoftwareSerial * serial );
#endif
    MBStatus_t process( void );
    void setRegisterMap( MBRegister_t * registerMap );
    MBStatus_t checkRegisterMap( void );

    /* Only for function code 8 (DIAGNOSTIC). */

    uint16_t diagRegGet( void );
    bool diagRegGet( size_t bit );
    bool diagRegSet( size_t bit );
    bool diagRegClear( size_t bit );
    void diagRegClear( void );

    /* Simplified API functions. */

    bool createRegister( MBAccess_t xAccess, uint16_t usAddress, size_t xNumber );
    bool createCoils( uint16_t address, size_t number );
    bool createInputResgisters( uint16_t address, size_t number );
    bool createHoldingRegisters( uint16_t address, size_t number );
    int32_t getCoil( uint16_t address );
    int32_t getInputResgister( uint16_t address );
    int32_t getHoldingRegister( uint16_t address );
    bool setCoil( uint16_t address, uint16_t value );
    bool setInputResgister( uint16_t address, uint16_t value );
    bool setHoldingRegister( uint16_t address, uint16_t value );

private:

    uint8_t ucServerId;
    MBServerState_t xState;
    void vSetState( MBServerState_t xStatePar );
    MBRegister_t * pxRegisterMap;
    size_t xRegisterMapIndex;
    MBStatus_t xCheckRequest( uint16_t usReqAddress, uint8_t ucReqFunctionCode );
    void vHandlerFC03_04( void );
    void vHandlerFC05( void );
    void vHandlerFC06( void );
    void vHandlerFC16( void );
    void vHandlerFC08( void );
    void vClearDiagnosticCounters( void );
    uint16_t usBusMessageCount;
    uint16_t usBusCommunicationErrorCount;
    uint16_t usServerExceptionErrorCount;
    uint16_t usServerMessageCount;
    uint16_t usServerNoResponseCount;
    uint16_t usServerNAKCount;
    uint16_t usServerBusyCount;
    uint16_t usBusCharacterOverrunCount;
    uint16_t usDiagnosticRegister;
    bool bListenOnlyMode;
#if( configSERVER_MULTI_ID == 1 )
    uint8_t ucIdMap[ configID_COUNT_MAX ];
    size_t xIdCount;
    void vSetIdMap( void );
#endif
    bool bCheckId( uint8_t ucId );
    size_t xRegisterMapSize;
    int32_t lGetRegister( uint16_t address );
    bool bSetRegister( uint16_t address, uint16_t value );
};
/*-----------------------------------------------------------*/

#if( configFC08 == 1 )
    #define vIncCPT( counter ) if( counter < 0xFFFF ) counter++
#if( configSFC11 == 1 )
    #define vIncCPT1()  vIncCPT( usBusMessageCount )
#else
    #define vIncCPT1()
#endif
#if( configSFC12 == 1 )
    #define vIncCPT2()  vIncCPT( usBusCommunicationErrorCount )
#else
    #define vIncCPT2()
#endif
#if( configSFC13 == 1 )
    #define vIncCPT3()  vIncCPT( usServerExceptionErrorCount )
#else
    #define vIncCPT3()
#endif
#if( configSFC14 == 1 )
    #define vIncCPT4()  vIncCPT( usServerMessageCount )
#else
    #define vIncCPT4()
#endif
#if( configSFC15 == 1 )
    #define vIncCPT5()  vIncCPT( usServerNoResponseCount )
#else
    #define vIncCPT5()
#endif
#if( configSFC16 == 1 )
    #define vIncCPT6()  vIncCPT( usServerNAKCount )
#else
    #define vIncCPT6()
#endif
#if( configSFC17 == 1 )
    #define vIncCPT7()  vIncCPT( usServerBusyCount )
#else
    #define vIncCPT7()
#endif
#if( configSFC18 == 1 )
    #define vIncCPT8()  vIncCPT( usBusCharacterOverrunCount )
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
