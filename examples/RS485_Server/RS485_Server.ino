////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        RS485_Server.ino
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

#define RS485_CTRL_PIN 2

/*----------------------------------------------------------------------------*/

SerialModbusServer ModbusServer;

/*----------------------------------------------------------------------------*/

void rs485Tx( void );
void rs485Rx( void );

/*----------------------------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    pinMode( RS485_CTRL_PIN, OUTPUT );

    ModbusServer.setSerialCtrl( rs485Tx, rs485Rx );
    ModbusServer.begin( 1, 9600 );

    ModbusServer.createHoldingRegister( 1000, 1 );
    ModbusServer.setRegister( 1000, 0 );
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
/*----------------------------------------------------------------------------*/

void rs485Tx( void )
{
    digitalWrite( RS485_CTRL_PIN, HIGH );
}
/*----------------------------------------------------------------------------*/

void rs485Rx( void )
{
    digitalWrite( RS485_CTRL_PIN, LOW );
}
