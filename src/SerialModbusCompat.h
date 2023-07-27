////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusCompat.h
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

#ifndef __SERIAL_MODBUS_COMPAT_H__
#define __SERIAL_MODBUS_COMPAT_H__

/*-----------------------------------------------------------*/

#include <Arduino.h>

/*-----------------------------------------------------------*/

/* This value for the Arduino default serial configuration is taken from the
source code (8 data bits, no parity, 1 stop bit). */
#define SERIAL_CONFIG_DEFAULT   SERIAL_8N1

/* Check if the architecture of the currently selected borad is supported. */
#if defined( ARDUINO_ARCH_AVR ) || \
    defined( ARDUINO_ARCH_MEGAAVR ) || \
    defined( ARDUINO_ARCH_SAMD ) || \
    defined( ARDUINO_ARCH_RENESAS )

    /* Check for the known boards that are not (!) compatible with the Arduino
    SoftwareSerial library - and if none of them is selected, the compatibility
    gets activated. */
    #if !defined( ARDUINO_SAMD_MKRZERO ) && \
        !defined( ARDUINO_SAMD_NANO_33_IOT )

        #define COMPAT_SOFTWARE_SERIAL

    #endif

    /* Check the type of the currently selected board and set the needed
    parameters if it is supported. */
    #if defined( ARDUINO_AVR_UNO ) || \
        defined( ARDUINO_AVR_MEGA2560 ) || \
        defined( ARDUINO_AVR_LEONARDO ) || \
        defined( ARDUINO_AVR_PRO )

        #define Serial_t    HardwareSerial

    #elif defined( ARDUINO_AVR_NANO_EVERY ) || \
          defined( ARDUINO_AVR_UNO_WIFI_REV2 )

        #define Serial_t    UartClass

    #elif defined( ARDUINO_SAMD_MKRZERO ) || \
          defined( ARDUINO_SAMD_NANO_33_IOT )

        #define Serial_t    Uart

    #elif defined( ARDUINO_MINIMA ) || \
          defined( ARDUINO_UNOWIFIR4 )

        #define Serial_t                UART
        #define SERIAL_PORT_HARDWARE    Serial1

    #else

        #warning The selected board has not been tested with this version of SerialModbus.

        /* For unknown boards we deactivate the SoftwareSerial compatibility. */
        #if defined( COMPAT_SOFTWARE_SERIAL )
            #undef COMPAT_SOFTWARE_SERIAL
        #endif

    #endif

#else

    #warning The selected board and architecture have not been tested with this version of SerialModbus.

#endif

/* If none of the needed defines is set, we try to use some assumed default
values for the Arduino serial port. */

#if !defined( Serial_t )
    #define Serial_t                HardwareSerial
#endif
#if !defined( SERIAL_PORT_HARDWARE )
    #define SERIAL_PORT_HARDWARE    Serial
#endif

/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_COMPAT_H__ */
