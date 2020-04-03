////////////////////////////////////////////////////////////////////////////////
/**
 * @file        BasicSerialMaster.ino
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

#include <SerialModbusMaster.h>

/*-----------------------------------------------------------*/

SerialModbusMaster Master;
    
uint16_t object[ 1 ] = { COIL_OFF };

void toggleCoil( void );
void toggleLed( void );

const MBRequest_t requestMap[] = {
    { 1, WRITE_SINGLE_COIL, 0x1000, object, 1, toggleCoil },
    { 1, WRITE_SINGLE_COIL, 0x2000, object, 1, toggleCoil },
    REQUEST_MAP_END
};
/*-----------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    Master.setRequestMap( requestMap );
    Master.begin( 19200, &Serial );
}
/*-----------------------------------------------------------*/

void loop( void )
{
    if( Master.processModbus() != OK )
    {
        toggleLed();
    }

    delay( 100 );
}
/*-----------------------------------------------------------*/

void toggleCoil( void )
{
    if( object[ 0 ] == COIL_OFF )
    {
        object[ 0 ] = COIL_ON;
    }
    else
    {
        object[ 0 ] = COIL_OFF;
    }
}
/*-----------------------------------------------------------*/

void toggleLed( void )
{
    if( digitalRead( LED_BUILTIN ) == LOW )
    {
        digitalWrite( LED_BUILTIN, HIGH );
    }
    else
    {
        digitalWrite( LED_BUILTIN, LOW );
    }
}
