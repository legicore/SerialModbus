////////////////////////////////////////////////////////////////////////////////
/**
 * @file        BasicSerialSlave.ino
 * 
 * @author      Martin Legleiter
 * 
 * @brief       TODO
 * 
 * @copyright   (c) 2018 Martin Legleiter
 * 
 * @license     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#include <SerialModbusSlave.h>

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

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    Slave.setRegisterMap( registerMap );
    Slave.begin( 1, 19200, &Serial );
}
/*-----------------------------------------------------------*/

void loop( void )
{
    Slave.processModbus();
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
