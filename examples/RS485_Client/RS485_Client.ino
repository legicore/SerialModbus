////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        RS485_Client.ino
 *
 * COPYRIGHT:   (C) 2026 Martin Legleiter
 *
 * LICENCE:     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <SerialModbusClient.h>

/*----------------------------------------------------------------------------*/

#define RS485_CTRL_PIN 2

/*----------------------------------------------------------------------------*/

SerialModbusClient ModbusClient;

/*----------------------------------------------------------------------------*/

void max485Tx( void );
void max485Rx( void );

/*----------------------------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    pinMode( RS485_CTRL_PIN, OUTPUT );

    ModbusClient.setSerialCtrl( max485Tx, max485Rx );
    ModbusClient.begin( 9600 );
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
/*----------------------------------------------------------------------------*/

void max485Tx( void )
{
    digitalWrite( RS485_CTRL_PIN, HIGH );
}
/*----------------------------------------------------------------------------*/

void max485Rx( void )
{
    digitalWrite( RS485_CTRL_PIN, LOW );
}
