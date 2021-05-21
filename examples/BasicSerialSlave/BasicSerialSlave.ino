////////////////////////////////////////////////////////////////////////////////
/**
 * @file        BasicSerialSlave.ino
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

#include <SerialModbusSlave.h>

/*-----------------------------------------------------------*/

SerialModbusSlave Slave;

uint16_t object1[ 1 ] = { 0 };
uint16_t object2[ 1 ] = { 0 };

void action1( void );
void action2( void );

const MBRegister_t registerMap[] = {
    { WR, 0x1000, object1, 1, action1 },
    { RD, 0x2000, object2, 1, action2 },
    REGISTER_MAP_END
};
/*-----------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    Slave.setRegisterMap( registerMap );
    Slave.begin( 1, 9600, &Serial );
}
/*-----------------------------------------------------------*/

void loop( void )
{
    Slave.processModbus();
}
/*-----------------------------------------------------------*/

void action1( void )
{
    digitalWrite( LED_BUILTIN, HIGH );
    object2[ 0 ] = object1[ 0 ] + 1;
}
/*-----------------------------------------------------------*/

void action2( void )
{
    digitalWrite( LED_BUILTIN, LOW );
}
