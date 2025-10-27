////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SerialModbusCompat.h
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

#ifndef __SERIAL_MODBUS_COMPAT_H__
#define __SERIAL_MODBUS_COMPAT_H__

/*----------------------------------------------------------------------------*/

#include <Arduino.h>

/*----------------------------------------------------------------------------*/

/* Check if the architecture of the currently selected borad is supported. */
#if defined( ARDUINO_ARCH_AVR ) || \
    defined( ARDUINO_ARCH_MEGAAVR ) || \
    defined( ARDUINO_ARCH_SAMD ) || \
    defined( ARDUINO_ARCH_RENESAS )

    /* Check the type of the currently selected board and set the needed
     * parameters if it is supported. */
    #if defined( ARDUINO_AVR_UNO ) || \
        defined( ARDUINO_AVR_MEGA2560 ) || \
        defined( ARDUINO_AVR_LEONARDO ) || \
        defined( ARDUINO_AVR_PRO )

        #define configMB_SERIAL         HardwareSerial
        #define configMB_SERIAL_SW      SoftwareSerial

    #elif defined( ARDUINO_AVR_NANO_EVERY ) || \
          defined( ARDUINO_AVR_UNO_WIFI_REV2 )

        #define configMB_SERIAL         UartClass
        #define configMB_SERIAL_SW      SoftwareSerial

    #elif defined( ARDUINO_SAMD_MKRZERO ) || \
          defined( ARDUINO_SAMD_MKRWIFI1010 ) || \
          defined( ARDUINO_SAMD_NANO_33_IOT )

        #define configMB_SERIAL         Uart

    #elif defined( ARDUINO_MINIMA ) || \
          defined( ARDUINO_UNOWIFIR4 )

        #define configMB_SERIAL         UART
        #define configMB_SERIAL_SW      SoftwareSerial

        #define SERIAL_PORT_HARDWARE    Serial1

    #else

        #warning The selected board has not been tested with this version of SerialModbus.

    #endif

#else

    #warning The selected board and architecture have not been tested with this version of SerialModbus.

#endif

/* If none of the needed defines is set, we try to use some assumed default
 * values for the Arduino serial port. */

#if !defined( configMB_SERIAL )

    #define configMB_SERIAL HardwareSerial

#endif
#if !defined( SERIAL_PORT_HARDWARE )

    #define SERIAL_PORT_HARDWARE Serial

#endif

/*----------------------------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_COMPAT_H__ */
