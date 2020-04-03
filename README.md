# SerialModbus

## Preface

I'm a embedded systems developer and as such I write most of my C and C++ code in a more traditional and/or standard C style. In my projects I work alot with small MCUs (Micro Controler Unit) like the Arduino UNO or the Arduino Mega 2560 which have a limited amount of memory and processing capabilities. Because of this I try to avoid the usage of language specific techniques that (could) produce a lot of "invisible" background code to save space and general capacities.

## Code Style

In past and ongoing projects I worked with the RTOS (Realtime Operatin System) [FreeRTOS](https://www.freertos.org/index.html) by Richard Barry. Its coding style is very clean, easy to read and based on the [MISRA C](https://www.misra.org.uk/Activities/MISRAC/tabid/160/Default.aspx) standard. I got used to it and write most of my code based on the FreeRTOS [Coding Standard and Style Guide](https://www.freertos.org/FreeRTOS-Coding-Standard-and-Style-Guide.html).

## General Information

### Official Specification

The library has been developed (and will be further improved/expanded) in strict compliance with the official Modbus specifications and guidlines.
* [MODBUS Application Protocol Specification](http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)
* [MODBUS over serial line specification and implementation guid](http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf)

### Supported Function Codes

* 3 ( READ_HOLDING_REGISTERS )
* 4 ( READ_INPUT_REGISTER )
* 5 ( WRITE_SINGLE_COIL )
* 6 ( WRITE_SINGLE_REGISTER )
* 16 ( WRITE_MULTIPLE_REGISTERS )

### Tested Boards

* Arduino UNO
* Arduino Mega 2650
* Arduino Leonardo
* Arduino Pro Mini (ATmega328P, 5V, 16MHz)

# Getting Started

## Installation

Download the **.zip* file from GitHub and unzip it. Since this is a Arduino library it just has to be copied into your local Arduino sketchbook libraries folder `<YOUR SKETCHBOOK LOCATION>/libraries/`.

## Configuration

The header file `SerialModbusConfig.h` holds serveral defines to configure the behavior and parameters of the library.

```C++
#define configMODE_RTU                  1
#define configMODE_ASCII                2
#define configMODE                      configMODE_RTU

#define configMAX_FRAME_SIZE            ( 64 )

#define configID_BROADCAST              ( ( uint8_t ) 0 )
#define configID_SLAVE_MAX              ( ( uint8_t ) 247 )

#define configASCII_INPUT_DELIMITER     ( '\n' )
```

## Usage

To use the library in your Arduino sketches you have to include the needed header file for the master or slave role and declare a corresponding object.

```C++
#incude <SerialModbusMaster.h>
SerialModbusMaster Master;
```

```C++
#incude <SerialModbusSlave.h>
SerialModbusSlave Slave;
```

### Timings

These timing values are the default timing values for the library. They will be initialized when a new object of the type `SerialModbusMaster` or `SerialModbusSlave` is declared. The defaut values are based on the official Modbus specification but could be changed in the setup phase ( or even during runtime ) via the API methods `setTurnaroundDelay()` and `setResponseTimeout()` ( see chapter *API* ).

```C++
#define configTURNAROUND_DELAY_US   200000
#define configRESPONSE_TIMEOUT_US   1000000
```

> For data rates lower than 19200 Baud the inter character timeout must be doubled. This could be done in the config header or via the `setInterCharacterTimeout()` methode in the code (which is the recommended way).

### Function Codes

There are already placeholders for all standard Modbus function codes defined in the configuration header. They are in the style of `configFC<XX>` and could have the value `1` ( include ), `0` ( exclude ) or `N` ( not implemented ). If they are configured as `0` ( or `N` ) the associated program code will be excluded from the compile process via preprocessor directives and will not be part of the resulting binary output.

For example, if the function code 16 ( WRITE_MULTIPLE_REGISTERS ) is not needed for an application you can set it to `0` and reduce the resulting binary output size.

```C++
#define N 0 /* Not implemented */

/* Bit Data Access */
#define configFC01      N   /* READ_COILS */
#define configFC02      N   /* READ_DISCRETE_INPUTS */
#define configFC05      1   /* WRITE_SINGLE_COIL */
#define configFC15      N   /* WRITE_MULTIPLE_COILS */

/* Word Data Access */
#define configFC03      1   /* READ_HOLDING_REGISTERS */
#define configFC04      1   /* READ_INPUT_REGISTER */
#define configFC06      1   /* WRITE_SINGLE_REGISTER */
#define configFC16      1   /* WRITE_MULTIPLE_REGISTERS */
#define configFC22      N   /* MASK_WRITE_REGISTER */
#define configFC23      N   /* READ_WRITE_MULTIPLE_REGISTERS */
#define configFC24      N   /* READ_FIFO_QUEUE */

/* File Record Data Access */
#define configFC20      N   /* READ_FILE_RECORD */
#define configFC21      N   /* WRITE_FILE_RECORD */

/* Diagnistics */
#define configFC07      N   /* READ_EXCEPTION_STATUS */
#define configFC08      0   /* DIAGNOSTIC */
#define configFC11      N   /* GET_COM_EVENT_COUNTER */
#define configFC12      N   /* GET_COM_EVENT_LOG */
#define configFC17      N   /* REPORT_SLAVE_ID */

#undef N /* Not implemented */
```

## API

The API (Application Programming Interface) was designed to be sleak, simple and (Arduino typical) very user friendly and easy to use even for beginners.

### Modbus Data

```C++
typedef struct MBRequest_s
{
    uint8_t id;
    uint8_t functionCode;
    uint16_t address;
    uint16_t * object;
    size_t objectSize;
    void (*action)( void );
}
MBRequest_t;

#define REQUEST_MAP_END { 0x00, 0x00, 0x0000, NULL, 0, NULL }
```

```C++
typedef struct MBRegister_s
{
    MBAccess_t access;
    uint16_t address;
    uint16_t * object;
    size_t objectSize;
    void (*action)( void );
}
MBRegister_t;

#define REGISTER_MAP_END { ( MBAccess_t ) 0b00, 0x0000, NULL, 0, NULL }
```

### Common Methods

> `void setSerialCtrl( void (*serialCtrlTx)( void ), void (*serialCtrlRx)( void ) );`
* This method sets control functions for the serial communication ( and is **mendatory** dependent on the type of the library integration and the used transceiver IC type ). Modbus is mostly used within RS-485 networks and the usual transceivers. The developer has to provide the control functions to set the transceiver to the appropriate RX or TX mode.
    * `serialCtrlTx` : Function pointer ( `void(*)( void )` ) to the applicable TX control function.
    * `serialCtrlRx` : Function pointer ( `void(*)( void )` ) to the applicable RX control function.

> `void setDataList( const MBData_t * dataList );`
* TODO
    * `dataList` : TODO

> `void setProcessLoopHook( void (*loopHookFunction)( void ) );`
* TODO
    * `loopHookFunction` : TODO

> `void setInterFrameDelay( uint32_t timeMs );`
* TODO
    * `timeMs` : TODO

> `MBStatus_t processModbus( void );`
* TODO

### Slave Specific Methodes

> `void begin( uint8_t slaveId, uint32_t baud, HardwareSerial * serial, uint8_t config );`
* TODO
    * `slaveId` : The desired slave ID.
    * `baud` : The value of the desired baud rate.
    * `serial` : A pointer to the desired Arduino (hardware) serial port.
        * Default value for Arduino UNO : `&Serial`.
        * Default value for Arduino Mega and Leonardo : `&Serial1`.
    * `config` : The configuration byte of the Arduino (hardware) serial port.
        * Default value : `SERIAL_8N1` (8 data bits, no parity, 1 stop bit).

> `void begin( uint8_t slaveId, uint32_t baud, SoftwareSerial * serial );`
* TODO
    * `slaveId` : The desired slave ID.
    * `baud` : The value of the desired baud rate.
    * `serial` : A pointer to the desired software serial port.

### Master Specific Methodes

> `void begin( uint32_t baud, HardwareSerial * serial, uint8_t config );`
* TODO
    * `baud` : The value of the desired baud rate.
    * `serial` : A pointer to the desired Arduino (hardware) serial port.
        * Default value for Arduino UNO : `&Serial`.
        * Default value for Arduino Mega and Leonardo : `&Serial1`.
    * `config` : The configuration byte of the Arduino (hardware) serial port.
        * Default value : `SERIAL_8N1` (8 data bits, no parity, 1 stop bit).

> `void begin( uint32_t baud, SoftwareSerial * serial );`
* TODO
    * `baud` : The value of the desired baud rate.
    * `serial` : A pointer to the desired software serial port.

> `MBStatus_t setRequest( MBData_t * request );`
* TODO
    * `request` : TODO

> `size_t getReplyDataSize( void );`
* TODO

> `size_t getReplyData( uint16_t * buffer, size_t bufferSize );`
* TODO
    * `buffer` : TODO
    * `bufferSize` : TODO

> `void setResponseTimeout( uint32_t timeMs );`
* Sets a new response timeout value.
    * `timeMs` : The new timeout value in milliseconds.

> `void setTurnaroundDelay( uint32_t timeMs );`
* Sets a new turnaround delay value.
    * `timeMs` : The new timeout value in milliseconds.

## Usage Hints

The following points are intended to prevent incorrect usage of the library, show possible error sources and avoid frustration and general errors.

### 1. Slaves Register Address Range

The developer is responsible for the distribution of the slave registers.
The following slave register map would be processed without errors but lead to faulty behavior.
```C++
const MBRegister_t xRegisterMap[] = {
    { WR, 0x1001, obj1, 3, NULL },
    { WR, 0x1002, obj2, 1, NULL }, // FAULTY ADDRESS !!!
    REGISTER_MAP_END
};
```
> The first entry has the address value 0x1001 and has a object size of 3 - this covers the address values 0x1001, 0x1002 and 0x1003.
The second entry address lies in the address range of the first entry and must at least have the address value 0x1004!

# Todo

- [x] Include support for SoftwareSerial;
- [x] Include support for HardwareSerial bit configuration;
- [ ] Finish implementation of function code 8 ( DIAGNOSTIC );

# License

> MIT License
>
> Copyright (c) 2020 Martin Legleiter
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all
> copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.
