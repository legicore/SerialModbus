////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusCompat.h
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

#ifndef __SERIAL_MODBUS_COMPAT_H__
#define __SERIAL_MODBUS_COMPAT_H__

/*-----------------------------------------------------------*/

#include <Arduino.h>

/*-----------------------------------------------------------*/

/* This value for the Arduino default serial configuration is taken from the
source code (8 data bits, no parity, 1 stop bit). */
#define SERIAL_CONFIG_DEFAULT SERIAL_8N1

/* Check the architecture, based on the architecture of the tested boards. */
#if defined( ARDUINO_ARCH_AVR ) || \
    defined( ARDUINO_ARCH_MEGAAVR ) || \
    defined( ARDUINO_ARCH_MBED_NANO ) || defined( ARDUINO_ARCH_MBED ) || \
    defined( ARDUINO_ARCH_SAMD )

    /* Check for the known boards that are not (!) compatible with the Arduino
    SoftwareSerial library, and if none of them is selected, the compatibility
    gets activated. */
    #if !defined( ARDUINO_NANO_RP2040_CONNECT ) && \
        !defined( ARDUINO_SAMD_MKRZERO )

        #define COMPAT_SOFTWARE_SERIAL

    #endif

    /* Check for the known/tested boards and set the Serial_t type to the
    associated Arduino serial type. */
    #if defined( ARDUINO_AVR_UNO ) || \
        defined( ARDUINO_AVR_MEGA2560 ) || \
        defined( ARDUINO_AVR_LEONARDO ) || \
        defined( ARDUINO_AVR_PRO ) || \
        defined( ARDUINO_NANO_RP2040_CONNECT ) || \
        defined( ARDUINO_SAMD_MKRZERO )

        #define Serial_t HardwareSerial

    #elif defined( ARDUINO_AVR_NANO_EVERY ) || \
          defined( ARDUINO_AVR_UNO_WIFI_REV2 )

        #define Serial_t UartClass

    #else
        /* The selected board has a known architecture, but has not yet been
        tested. So the assumed default values for the Arduino serial port will
        be set, and the SoftwareSerial compatibility gets deactivated. */

        #define Serial_t HardwareSerial
        #if !defined( SERIAL_PORT_HARDWARE )
            #define SERIAL_PORT_HARDWARE Serial
        #endif

        #if defined( COMPAT_SOFTWARE_SERIAL )
            #undef COMPAT_SOFTWARE_SERIAL
        #endif

        #warning The selected board has not yet been tested with this version of SerialModbus!

    #endif

#else
    /* The selected board and its architecture have not yet been tested. So the
    assumed default values for the Arduino serial port will be set. */

    #define Serial_t HardwareSerial
    #if !defined( SERIAL_PORT_HARDWARE )
        #define SERIAL_PORT_HARDWARE Serial
    #endif

    #warning The selected board and its architecture have not yet been tested with this version of SerialModbus!

#endif

/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_COMPAT_H__ */
