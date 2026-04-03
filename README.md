# SerialModbus

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
* 8 ( DIAGNOSTIC )
* 16 ( WRITE_MULTIPLE_REGISTERS )

### Tested Boards

* Arduino UNO R3
* Arduino UNO R4 WiFi
* Arduino UNO R4 Minima
* Arduino UNO WiFi Rev2
* Arduino Leonardo
* Arduino Mega 2560 Rev3
* Arduino Pro Mini (ATmega328P, 5V, 16MHz)
* Arduino Nano
* Arduino Nano 33 IoT
* Arduino Nano Every
* Arduino Nano RP2040 Connec
* Arduino Nano ESP32
* Arduino MKR WiFi 1010

# Getting Started

## Installation

Download the **.zip* file from GitHub and unzip it. Since this is a Arduino library it just has to be copied into your local Arduino sketchbook libraries folder `<YOUR SKETCHBOOK LOCATION>/libraries/`.

## Configuration

The header file `SerialModbusConfig.h` holds serveral defines to configure the function of the library.

```C++
#define configMB_MODE_RTU               1
#define configMB_MODE_ASCII             2
#define configMB_MODE                   configMB_MODE_RTU

#define configMB_FRAME_LEN_MIN          3
#define configMB_FRAME_LEN_MAX          64

/* This value for the Arduino default serial configuration is taken from the 
 * source code (8 data bits, no parity, 1 stop bit). */ 
#define configMB_SERIAL_CONF_DEFAULT    SERIAL_8N1

/*----------------------------------------------------------------------------*/

#define configMB_ID_BROADCAST           0
#define configMB_ID_SERVER_MAX          247

#define configMB_ASCII_INPUT_DELIMITER  '\n'

#define configMB_TURNAROUND_DELAY_MS    200
#define configMB_RESPONSE_TIMEOUT_MS    1000

/*----------------------------------------------------------------------------*/

#define configMB_PROCESS_LOOP_HOOK      0

#define configMB_EXT_EXCEPTION_CODES    0

#define configMB_SERVER_MULTI_ID        0
#define configMB_ID_COUNT_MAX           8
```

### Process Loop Hook

TODO

### Extended Exception Codes

TODO

### Multi ID Support

TODO

## Usage

To use the library in your Arduino sketches you have to include the needed header file for the client or server role and declare a corresponding object.

```C++
#incude <SerialModbusClient.h>
SerialModbusClient MBClient;
```

```C++
#incude <SerialModbusServer.h>
SerialModbusServer MBServer;
```

# Todo

- [x] Include support for SoftwareSerial;
- [x] Include support for HardwareSerial bit configuration;
- [x] Finish implementation of function code 8 ( DIAGNOSTIC );
