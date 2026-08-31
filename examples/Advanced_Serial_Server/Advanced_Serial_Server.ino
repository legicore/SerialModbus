////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        Advanced_Serial_Server.ino
 *
 * COPYRIGHT:   (C) 2026 Martin Legleiter
 *
 * LICENCE:     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <SerialModbusServer.h>

/*----------------------------------------------------------------------------*/

SerialModbusServer ModbusServer;
uint16_t data = 0;

/*----------------------------------------------------------------------------*/

void callback( void );

/*----------------------------------------------------------------------------*/

MB_Register_t registerMap[] = {
    { MB_WO, 1000, &data, 1, NULL },
    { MB_RO, 2000, &data, 1, callback },
    MB_REGISTER_MAP_END
};
/*----------------------------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    /* INFO:    The begin() method has two more parameters with default values.
     *          The following line shows the optional parameters to set the 
     *          serial interface we want to use and its configuration.
     *          e.g.: ModbusServer.begin( 1, 9600, &Serial, SERIAL_8N1 ); */
    ModbusServer.begin( 1, 9600 );

    ModbusServer.setRegisterMap( registerMap );
    if( ModbusServer.checkRegisterMap() != MB_OK )
    {
        /* React to an error in the register map. */
    }

    /* The following functions can be used to get and set timings.
     *
     * INFO:    By defaulty the Inter Character Timeout and the
     *          Inter Frame Delay are calculated automatically, based on the
     *          baud rate and the serial bit configuration. */

    // uint32_t interCharacterTimeout = ModbusServer.getInterCharacterTimeout();
    // ModbusServer.setInterFrameDelay( interCharacterTimeout );

    // uint32_t interFrameDelay = ModbusServer.getInterFrameDelay();
    // ModbusServer.setInterCharacterTimeout( interFrameDelay );
}
/*----------------------------------------------------------------------------*/

void loop( void )
{
    MB_Status_t result = MB_OK;

    result = ModbusServer.process();
    if( result != MB_OK )
    {
        /* React to a possible processing error - e.g. in conjunction with one
         * of the following methods:
         *      - ModbusServer.getExceptionString( result );
         *      - ModbusServer.getLastExceptionString();
         *      - ModbusServer.getLastException(); */
    }
}
/*----------------------------------------------------------------------------*/

void callback( void )
{
    if( data == 1 )
    {
        digitalWrite( LED_BUILTIN, HIGH );
    }
    else
    {
        digitalWrite( LED_BUILTIN, LOW );
    }
}
