////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusSlave.h
 * 
 * @author      Martin Legleiter
 * 
 * @brief       TODO
 * 
 * @copyright   (c) 2018 Martin Legleiter
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

#include "SerialModbusBase.h"

#include <Arduino.h>
#include <SoftwareSerial.h>

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
    MBAccess_t access;
    uint16_t address;
    uint16_t * object;
    size_t objectSize;
    void (*action)( void );
}
MBRegister_t;

/** TODO */
#define REGISTER_MAP_END { NA, 0x0000, NULL, 0, NULL }

/*-----------------------------------------------------------*/

/** TODO */
class SerialModbusSlave : public SerialModbusBase
{
public:

    SerialModbusSlave();
#if defined( __AVR_ATmega640__ ) || defined( __AVR_ATmega1280__ ) || defined( __AVR_ATmega1281__ ) || defined( __AVR_ATmega2560__ ) || defined( __AVR_ATmega2561__ ) || ( __AVR_ATmega328P__ ) || defined( __AVR_ATmega168__ ) || defined( __AVR_ATmega8__ )
    void begin( uint8_t slaveId, uint32_t baud, HardwareSerial * serial = &Serial, uint8_t config = configUART_SETTINGS );
#elif defined( __AVR_ATmega32U4__ ) || defined( __AVR_ATmega16U4__ )
    void begin( uint8_t slaveId, uint32_t baud, HardwareSerial * serial = &Serial1, uint8_t config = configUART_SETTINGS );
#endif
    void begin( uint8_t slaveId, uint32_t baud, SoftwareSerial * serial );
    MBStatus_t processModbus( void );
    void setRegisterMap( const MBRegister_t * registerMap );
#if( configFC08 == 1 )
    uint16_t diagRegGet( void );
    bool diagRegGet( size_t bit );
    bool diagRegSet( size_t bit );
    bool diagRegClear( size_t bit );
    void diagRegClear( void );
#endif

private:

    uint8_t ucSlaveId;
    MBSlaveState_t xState;
    void vSetState( MBSlaveState_t xStatePar );
    const MBRegister_t * pxRegisterMap;
    size_t xRegisterMapIndex;
    MBStatus_t xCheckRequest( uint16_t usReqAddress, uint8_t ucReqFunctionCode );
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
    void vHandler08( void );
    void vClearDiagnosticCounters( void );
    uint16_t usBusMessageCount;
    uint16_t usBusCommunicationErrorCount;
    uint16_t usExceptionErrorCount;
    uint16_t usSlaveMessageCount;
    uint16_t usSlaveNoResponseCount;
    uint16_t usSlaveNAKCount;
    uint16_t usSlaveBusyCount;
    uint16_t usBusCharacterOverrunCount;
    uint16_t usDiagnosticRegister;
#endif
    bool bListenOnlyMode;
};
/*-----------------------------------------------------------*/

#if( configFC08 == 1 )
    #define incCPT( x ) if( x < 0xFFFF ) { x++; }
    #define incCPT1()   incCPT( usBusMessageCount )
    #define incCPT2()   incCPT( usBusCommunicationErrorCount )
    #define incCPT3()   incCPT( usExceptionErrorCount )
    #define incCPT4()   incCPT( usSlaveMessageCount )
    #define incCPT5()   incCPT( usSlaveNoResponseCount )
    #define incCPT6()   incCPT( usSlaveNAKCount )
    #define incCPT7()   incCPT( usSlaveBusyCount )
    #define incCPT8()   incCPT( usBusCharacterOverrunCount )
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
