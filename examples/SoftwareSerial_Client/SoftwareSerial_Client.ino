////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        SoftwareSerial_Client.ino
 *
 * COPYRIGHT:   (C) 2026 Martin Legleiter
 *
 * LICENCE:     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <SerialModbusClient.h>
#include <SoftwareSerial.h>

/*----------------------------------------------------------------------------*/

SerialModbusClient ModbusClient;
SoftwareSerial SerialSW( 10, 11 );

/*----------------------------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    ModbusClient.begin( 9600, &SerialSW );
}
/*----------------------------------------------------------------------------*/

void loop( void )
{
    uint16_t data = 0;

    ModbusClient.writeSingleRegister( 1, 1000, 1 );
    ModbusClient.readHoldingRegister( 1, 1000, &data );

    if( data == 1 )
    {
        digitalWrite( LED_BUILTIN, HIGH );
    }

    delay( 1000 );

    ModbusClient.writeSingleRegister( 1, 1000, 0 );
    ModbusClient.readHoldingRegister( 1, 1000, &data );

    if( data == 0 )
    {
        digitalWrite( LED_BUILTIN, LOW );
    }

    delay( 1000 );
}
