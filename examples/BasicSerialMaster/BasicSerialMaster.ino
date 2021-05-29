////////////////////////////////////////////////////////////////////////////////
/**
 * @file        BasicSerialMaster.ino
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

#include <SerialModbusMaster.h>

/*-----------------------------------------------------------*/

SerialModbusMaster Master;
    
uint16_t object1[ 1 ] = { 0 };
uint16_t object2[ 1 ] = { 0 };

void action1( void );
void action2( void );

const MBRequest_t requestMap[] = {
    { 1, WRITE_SINGLE_REGISTER, 0x1000, object1, 1, action1 },
    { 1, READ_INPUT_REGISTERS,  0x2000, object2, 1, action2 },
    REQUEST_MAP_END
};
/*-----------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    Master.setRequestMap( requestMap );
    Master.begin( 9600 );
}
/*-----------------------------------------------------------*/

void loop( void )
{
    Master.processModbus();
    delay( 100 );
}
/*-----------------------------------------------------------*/

void action1( void )
{
    digitalWrite( LED_BUILTIN, HIGH );
}
/*-----------------------------------------------------------*/

void action2( void )
{
    if( object2[ 0 ] == object1[ 0 ] + 1 )
    {
        digitalWrite( LED_BUILTIN, LOW );
        object1[ 0 ] = object2[ 0 ];
    }
}
