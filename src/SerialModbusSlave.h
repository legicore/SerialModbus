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

#include "SerialModbusBase.h"

#include <Arduino.h>
#if !defined( ARDUINO_ARCH_RP2040 )
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
#if defined( __AVR_ATmega640__  ) || defined( __AVR_ATmega1280__ ) || defined( __AVR_ATmega1281__ ) || defined( __AVR_ATmega2560__ ) || defined( __AVR_ATmega2561__ ) || \
    defined( __AVR_ATmega328P__ ) || defined( __AVR_ATmega168__  ) || defined( __AVR_ATmega8__    ) || \
    defined( __AVR_ATmega32U4__ ) || defined( __AVR_ATmega16U4__ ) || \
    defined( ARDUINO_ARCH_RP2040 )
    bool begin( uint8_t slaveId, uint32_t baud, HardwareSerial * serial );
    bool begin( uint8_t slaveId, uint32_t baud, HardwareSerial * serial, uint8_t config );
#elif defined( __AVR_ATmega4809__ )
    bool begin( uint8_t slaveId, uint32_t baud, UartClass * serial );
    bool begin( uint8_t slaveId, uint32_t baud, UartClass * serial, uint32_t config );
#endif
#if !defined( ARDUINO_ARCH_RP2040 )
    bool begin( uint8_t slaveId, uint32_t baud, SoftwareSerial * serial );
#endif
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
    void vHandlerFC03_04( void );
#endif
#if( configFC05 == 1 )
    void vHandlerFC05( void );
#endif
#if( configFC06 == 1 )
    void vHandlerFC06( void );
#endif
#if( configFC16 == 1 )
    void vHandlerFC16( void );
#endif
#if( configFC08 == 1 )
    void vHandlerFC08( void );
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
    #define vIncCPT( counter ) if( counter < 0xFFFF ) counter++
    #define vIncCPT1()  vIncCPT( usBusMessageCount )
    #define vIncCPT2()  vIncCPT( usBusCommunicationErrorCount )
    #define vIncCPT3()  vIncCPT( usExceptionErrorCount )
    #define vIncCPT4()  vIncCPT( usSlaveMessageCount )
    #define vIncCPT5()  vIncCPT( usSlaveNoResponseCount )
    #define vIncCPT6()  vIncCPT( usSlaveNAKCount )
    #define vIncCPT7()  vIncCPT( usSlaveBusyCount )
    #define vIncCPT8()  vIncCPT( usBusCharacterOverrunCount )
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
