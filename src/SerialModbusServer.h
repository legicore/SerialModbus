////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SerialModbusServer.h
 * 
 * AUTHOR:      Martin Legleiter
 * 
 * BRIEF:       TODO
 * 
 * COPYRIGHT:   (C) 2025 Martin Legleiter
 * 
 * LICENCE:     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_SERVER_H__
#define __SERIAL_MODBUS_SERVER_H__

/*----------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SerialModbusCompat.h"
#include "SerialModbusBase.h"

#include <Arduino.h>
#if defined( configMB_SERIAL_SW )
    #include <SoftwareSerial.h>
#endif

/*----------------------------------------------------------------------------*/

enum MB_ServerState_e
{
    SERVER_IDLE,
    CHECKING_REQUEST,
    PROCESSING_REQUIRED_ACTION,
    FORMATTING_NORMAL_REPLY,
    FORMATTING_ERROR_REPLY
};

typedef enum MB_ServerState_e MB_ServerState_t;

/*----------------------------------------------------------------------------*/

enum MB_AddressType_e
{
    ADDR_NO_DATA          = 0,  /* No data access */
    ADDR_COIL             = 1,  /* Bit access read and write */
    ADDR_DISCRETE_INPUT   = 2,  /* Bit access read only */
    ADDR_HOLDING_REGISTER = 3,  /* Word access read and write */
    ADDR_INPUT_REGISTER   = 4   /* Word access read only */
};

typedef enum MB_AddressType_e MB_AddrType_t;

/*----------------------------------------------------------------------------*/

struct MB_Address_s
{
#if( configMB_SERVER_MULTI_ID == 1 )
    uint8_t id;
#endif
    MB_AddrType_t addressType;
    uint16_t address;
    void * data;
    size_t dataSize;
    MB_Callback_f callback;
};

typedef struct MB_Address_s MB_Address_t;

#define ADDR_MAP_END_ID              0xFF
#define ADDR_MAP_END_ADDRESS_TYPE    ADDR_NO_DATA
#define ADDR_MAP_END_ADDRESS         0xFFFF
#define ADDR_MAP_END_DATA            NULL
#define ADDR_MAP_END_DATA_SIZE       0
#define ADDR_MAP_END_CALLBACK        NULL

#if( configMB_SERVER_MULTI_ID == 0 )

    #define MB_ADDRESS_MAP_END      { ADDR_MAP_END_ADDRESS_TYPE, \
                                      ADDR_MAP_END_ADDRESS,      \
                                      ADDR_MAP_END_DATA,         \
                                      ADDR_MAP_END_DATA_SIZE,    \
                                      ADDR_MAP_END_CALLBACK }

    #define IS_ADDRESS_MAP_END( ENTRY )     ( ( ( ENTRY ).addressType == ADDR_MAP_END_ADDRESS_TYPE ) && \
                                              ( ( ENTRY ).address     == ADDR_MAP_END_ADDRESS      ) && \
                                              ( ( ENTRY ).data        == ADDR_MAP_END_DATA         ) && \
                                              ( ( ENTRY ).dataSize    == ADDR_MAP_END_DATA_SIZE    ) && \
                                              ( ( ENTRY ).callback    == ADDR_MAP_END_CALLBACK     ) )

#else

    #define MB_ADDRESS_MAP_END      { ADDR_MAP_END_ID,           \
                                      ADDR_MAP_END_ADDRESS_TYPE, \
                                      ADDR_MAP_END_ADDRESS,      \
                                      ADDR_MAP_END_DATA,         \
                                      ADDR_MAP_END_DATA_SIZE,    \
                                      ADDR_MAP_END_CALLBACK }

    #define IS_ADDRESS_MAP_END( ENTRY )     ( ( ( ENTRY ).id          == ADDR_MAP_END_ID           ) && \
                                              ( ( ENTRY ).addressType == ADDR_MAP_END_ADDRESS_TYPE ) && \
                                              ( ( ENTRY ).address     == ADDR_MAP_END_ADDRESS      ) && \
                                              ( ( ENTRY ).data        == ADDR_MAP_END_DATA         ) && \
                                              ( ( ENTRY ).dataSize    == ADDR_MAP_END_DATA_SIZE    ) && \
                                              ( ( ENTRY ).callback    == ADDR_MAP_END_CALLBACK     ) )

#endif
/*----------------------------------------------------------------------------*/

class SerialModbusServer : public SerialModbusBase
{
public:

    SerialModbusServer();
    bool begin( uint8_t id, uint32_t baud, MB_Serial_t * serial = &SERIAL_PORT_HARDWARE, uint32_t config = configMB_SERIAL_CONF_DEFAULT );
#if defined( configMB_SERIAL_SW )
    bool begin( uint8_t id, uint32_t baud, MB_SWSerial_t * serial );
#endif
    MB_Status_t process( void );
    bool setAddressMap( MB_Address_t * addressMap );
    bool resetAddressMap( void );
    MB_Address_t * getAddressMap( void );
    MB_Status_t checkAddressMap( void );

    /* Only for function code 8 (MB_DIAGNOSTIC). */

    uint16_t diagRegGet( void );
    bool diagRegGet( size_t bit );
    bool diagRegSet( size_t bit );
    bool diagRegClear( size_t bit );
    void diagRegClear( void );

    /* Simplified API functions. */

#if( configMB_SERVER_MULTI_ID == 0 )
    bool createCoil( uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getCoil( uint16_t address, uint16_t * data );
    bool setCoil( uint16_t address, uint16_t value );

    bool createDiscreteInput( uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getDiscreteInput( uint16_t address, uint16_t * data );
    bool setDiscreteInput( uint16_t address, uint16_t value );

    bool createHoldingRegister( uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getHoldingRegister( uint16_t address, uint16_t * data );
    bool setHoldingRegister( uint16_t address, uint16_t value );

    bool createInputRegister( uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getInputRegister( uint16_t address, uint16_t * data );
    bool setInputRegister( uint16_t address, uint16_t value );
#else
    bool createCoil( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getCoil( uint8_t id, uint16_t address, uint16_t * data );
    bool setCoil( uint8_t id, uint16_t address, uint16_t value );

    bool createDiscreteInput( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getDiscreteInput( uint8_t id, uint16_t address, uint16_t * data );
    bool setDiscreteInput( uint8_t id, uint16_t address, uint16_t value );

    bool createHoldingRegister( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getHoldingRegister( uint8_t id, uint16_t address, uint16_t * data );
    bool setHoldingRegister( uint8_t id, uint16_t address, uint16_t value );

    bool createInputRegister( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getInputRegister( uint8_t id, uint16_t address, uint16_t * data );
    bool setInputRegister( uint8_t id, uint16_t address, uint16_t value );
#endif

private:

    uint8_t ucServerId;
    MB_ServerState_t xState;
    void vSetState( MB_ServerState_t xStatePar );
    MB_Status_t xExceptionStatus;
    MB_Status_t xSetException( MB_Exception_t xException );
    MB_Address_t * pxAddressMap;
    size_t xAddressMapIndex;
    MB_Status_t xCheckRequest( uint16_t usReqAddress, uint8_t ucReqFunctionCode );
    void vHandlerFC03_04( void );
    void vHandlerFC05( void );
    void vHandlerFC06( void );
    void vHandlerFC07( void );
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
#if( configMB_SERVER_MULTI_ID == 1 )
    uint8_t ucIdMap[ configMB_ID_COUNT_MAX ];
    size_t xIdCount;
    void vSetIdMap( void );
#endif
    bool bCheckId( uint8_t ucId );
    size_t xAddressMapSize;
    bool bClearAddressMapEntry( MB_Address_t * pxAddressMapEntry );
    bool bAddressMapLock_sAPI;
    bool bAddressMapLock;
    bool bFindAddress( uint8_t ucId, MB_AddrType_t xAddressType, uint16_t usAddress );

    /* Simplified API functions. */

    bool bCreateAddress( uint8_t id, MB_AddrType_t addressType, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool bGetAddressBit( uint8_t id, MB_AddrType_t addressType, uint16_t address, uint16_t * data );
    bool bSetAddressBit( uint8_t id, MB_AddrType_t addressType, uint16_t address, uint16_t value );
    bool bGetAddressWord( uint8_t id, MB_AddrType_t addressType, uint16_t address, uint16_t * data );
    bool bSetAddressWord( uint8_t id, MB_AddrType_t addressType, uint16_t address, uint16_t value );

#if( configMB_SERVER_MULTI_ID == 0 )
    bool createCoil( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getCoil( uint8_t id, uint16_t address, uint16_t * data );
    bool setCoil( uint8_t id, uint16_t address, uint16_t value );

    bool createDiscreteInput( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getDiscreteInput( uint8_t id, uint16_t address, uint16_t * data );
    bool setDiscreteInput( uint8_t id, uint16_t address, uint16_t value );

    bool createHoldingRegister( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getHoldingRegister( uint8_t id, uint16_t address, uint16_t * data );
    bool setHoldingRegister( uint8_t id, uint16_t address, uint16_t value );

    bool createInputRegister( uint8_t id, uint16_t address, size_t dataSize, MB_Callback_f callback = NULL );
    bool getInputRegister( uint8_t id, uint16_t address, uint16_t * data );
    bool setInputRegister( uint8_t id, uint16_t address, uint16_t value );
#endif
};
/*----------------------------------------------------------------------------*/

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
/*----------------------------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_SERVER_H__ */
