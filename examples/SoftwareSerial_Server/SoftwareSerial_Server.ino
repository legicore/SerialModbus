////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SoftwareSerial_Server.ino
 *
 * COPYRIGHT:   (C) 2026 Martin Legleiter
 *
 * LICENCE:     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <SerialModbusServer.h>
#include <SoftwareSerial.h>

/*----------------------------------------------------------------------------*/

SerialModbusServer ModbusServer;
SoftwareSerial SerialSW( 10, 11 );

/*----------------------------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    ModbusServer.begin( 1, 9600, &SerialSW );

    ModbusServer.createHoldingRegisters( 1000, 1 );
}
/*----------------------------------------------------------------------------*/

void loop( void )
{
    uint16_t data = 0;

    ModbusServer.process();

    ModbusServer.getRegister( 1000, &data );
    if( data == 1 )
    {
        digitalWrite( LED_BUILTIN, HIGH );
    }
    else
    {
        digitalWrite( LED_BUILTIN, LOW );
    }
}
