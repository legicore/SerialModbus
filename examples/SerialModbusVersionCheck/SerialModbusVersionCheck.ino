////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusVersionCheck.ino
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

#include <SerialModbusVersion.h>

/*-----------------------------------------------------------*/

void setup( void )
{
    Serial.begin( 9600 );
}
/*-----------------------------------------------------------*/

void loop( void )
{
    Serial.print( "MAJOR : " );
    Serial.println( SERIALMODBUS_VERSION_MAJOR );
    Serial.print( "MINOR : " );
    Serial.println( SERIALMODBUS_VERSION_MINOR );
    Serial.print( "PATCH : " );
    Serial.println( SERIALMODBUS_VERSION_PATCH );

    Serial.print( "HEX   : 0x" );
    Serial.println( SERIALMODBUS_VERSION_HEX, HEX );

    Serial.print( SERIALMODBUS_VERSION_STR );
    Serial.print( " == 1.0.0 : " );
    if( SERIALMODBUS_VERSION_CHK_EQ( 1, 0, 0 ) == true )
        Serial.println( "true" );
    else
        Serial.println( "false" );

    Serial.print( SERIALMODBUS_VERSION_STR );
    Serial.print( " != 1.2.3 : " );
    if( SERIALMODBUS_VERSION_CHK_NE( 1, 2, 3 ) == true )
        Serial.println( "true" );
    else
        Serial.println( "false" );

    Serial.print( SERIALMODBUS_VERSION_STR );
    Serial.print( " <  1.1.0 : " );
    if( SERIALMODBUS_VERSION_CHK_LT( 1, 1, 0 ) == true )
        Serial.println( "true" );
    else
        Serial.println( "false" );

    Serial.print( SERIALMODBUS_VERSION_STR );
    Serial.print( " >  0.9.0 : " );
    if( SERIALMODBUS_VERSION_CHK_GT( 0, 9, 0 ) == true )
        Serial.println( "true" );
    else
        Serial.println( "false" );

    Serial.print( SERIALMODBUS_VERSION_STR );
    Serial.print( " <= 1.0.0 : " );
    if( SERIALMODBUS_VERSION_CHK_LTOE( 1, 0, 0 ) == true )
        Serial.println( "true" );
    else
        Serial.println( "false" );

    Serial.print( SERIALMODBUS_VERSION_STR );
    Serial.print( " >= 1.0.0 : " );
    if( SERIALMODBUS_VERSION_CHK_GTOE( 1, 0, 0 ) == true )
        Serial.println( "true" );
    else
        Serial.println( "false" );

    Serial.println();
    delay( 1000 );
}
