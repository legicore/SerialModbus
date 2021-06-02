////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusSlave.h
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

#ifndef __SERIAL_MODBUS_SLAVE_H__
#define __SERIAL_MODBUS_SLAVE_H__

/*-----------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusConfig.h"
#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"

#include <Arduino.h>
#if defined( COMPAT_SOFTWARE_SERIAL )
    #include <SoftwareSerial.h>
#endif

/*-----------------------------------------------------------*/

/** TODO */
typedef enum MBSlaveState_e
{
    SLAVE_IDLE,
    CHECKING_REQUEST,
    PROCESSING_REQUIRED_ACTION,
    FORMATTING_NORMAL_REPLY,
    FORMATTING_ERROR_REPLY
}
MBSlaveState_t;

/** TODO */
typedef enum MBAccess_e
{
    NA = 0b00,  /** No Access. */
    RD = 0b01,  /** Read only. */
    WR = 0b10,  /** Write only. */
    RW = 0b11   /** Read and write. */
}
MBAccess_t;

/** TODO */
typedef struct MBRegister_s
{
#if( configSLAVE_MULTI_ID == 1 )
    uint8_t id;
#endif
    MBAccess_t access;
    uint16_t address;
    uint16_t * object;
    size_t objectSize;
    void (*action)( void );
}
MBRegister_t;

/** TODO */
#if( configSLAVE_MULTI_ID == 1 )
    #define REGISTER_MAP_END { 0x00, NA, 0x0000, NULL, 0, NULL }
#else
    #define REGISTER_MAP_END { NA, 0x0000, NULL, 0, NULL }
#endif

/*-----------------------------------------------------------*/

/** TODO */
class SerialModbusSlave : public SerialModbusBase
{
public:

    SerialModbusSlave();
    bool begin( uint8_t slaveId, uint32_t baud, Serial_t * serial = &SERIAL_PORT_HARDWARE, uint32_t config = SERIAL_CONFIG_DEFAULT );
#if defined( COMPAT_SOFTWARE_SERIAL )
    bool begin( uint8_t slaveId, uint32_t baud, SoftwareSerial * serial );
#endif
    MBStatus_t processModbus( void );
    void setRegisterMap( const MBRegister_t * registerMap );
    uint16_t diagRegGet( void );
    bool diagRegGet( size_t bit );
    bool diagRegSet( size_t bit );
    bool diagRegClear( size_t bit );
    void diagRegClear( void );

private:

    uint8_t ucSlaveId;
    MBSlaveState_t xState;
    void vSetState( MBSlaveState_t xStatePar );
    const MBRegister_t * pxRegisterMap;
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
    uint16_t usSlaveExceptionErrorCount;
    uint16_t usSlaveMessageCount;
    uint16_t usSlaveNoResponseCount;
    uint16_t usSlaveNAKCount;
    uint16_t usSlaveBusyCount;
    uint16_t usBusCharacterOverrunCount;
    uint16_t usDiagnosticRegister;
    bool bListenOnlyMode;
#if( configSLAVE_MULTI_ID == 1 )
    uint8_t ucIdMap[ configMAX_ID_COUNT ];
    size_t xIdCount;
    void vSetIdMap( void );
    bool bCheckId( uint8_t ucId );
#endif
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
    #define vIncCPT3()  vIncCPT( usSlaveExceptionErrorCount )
#else
    #define vIncCPT3()
#endif
#if( configSFC14 == 1 )
    #define vIncCPT4()  vIncCPT( usSlaveMessageCount )
#else
    #define vIncCPT4()
#endif
#if( configSFC15 == 1 )
    #define vIncCPT5()  vIncCPT( usSlaveNoResponseCount )
#else
    #define vIncCPT5()
#endif
#if( configSFC16 == 1 )
    #define vIncCPT6()  vIncCPT( usSlaveNAKCount )
#else
    #define vIncCPT6()
#endif
#if( configSFC17 == 1 )
    #define vIncCPT7()  vIncCPT( usSlaveBusyCount )
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

#endif /* __SERIAL_MODBUS_SLAVE_H__ */
