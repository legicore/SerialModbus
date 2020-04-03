////////////////////////////////////////////////////////////////////////////////
/**
 * @file        BasicRS485Slave.ino
 * 
 * @author      Martin Legleiter
 * 
 * @brief       TODO
 * 
 * @copyright   (c) 2020 Martin Legleiter
 * 
 * @license     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <SerialModbusSlave.h>

/*-----------------------------------------------------------*/

#define MAX485_CTRL_PIN 2

/*-----------------------------------------------------------*/

SerialModbusSlave Slave;
    
uint16_t object[ 1 ] = { COIL_OFF };

void ledOn( void );
void ledOff( void );

const MBRegister_t registerMap[] = {
    { WR, 0x1000, object, 1, ledOn },
    { WR, 0x2000, object, 1, ledOff },
    REGISTER_MAP_END
};
/*-----------------------------------------------------------*/

void max485Tx( void );
void max485Rx( void );

/*-----------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    pinMode( MAX485_CTRL_PIN, OUTPUT );

    Slave.setSerialCtrl( &max485Tx, &max485Rx );
    Slave.setRegisterMap( registerMap );
    Slave.begin( 1, 19200, &Serial );
}
/*-----------------------------------------------------------*/

void loop( void )
{
    Slave.processModbus();
}
/*-----------------------------------------------------------*/

void max485Tx( void )
{
    digitalWrite( MAX485_CTRL_PIN, HIGH );
}
/*-----------------------------------------------------------*/

void max485Rx( void )
{
    digitalWrite( MAX485_CTRL_PIN, LOW );
}
/*-----------------------------------------------------------*/

void ledOn( void )
{
    digitalWrite( LED_BUILTIN, HIGH );
}
/*-----------------------------------------------------------*/

void ledOff( void )
{
    digitalWrite( LED_BUILTIN, LOW );
}
