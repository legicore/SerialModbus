////////////////////////////////////////////////////////////////////////////////
/*
 * FILE:        Advanced_Serial_Client.ino
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

SerialModbusClient ModbusClient;

uint16_t data = 0;

/*----------------------------------------------------------------------------*/

void callback( void );

/*----------------------------------------------------------------------------*/

MB_Request_t requestMap[] = {
    { 1, FC_WRITE_SINGLE_REGISTER,  1000, &data, 1, NULL },
    { 1, FC_READ_HOLDING_REGISTERS, 2000, &data, 1, callback },
    MB_REQUEST_MAP_END
};
/*----------------------------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    ModbusClient.begin( 9600 );

    /* The following functions can be used to get and set various timings.
     *
     * INFO:    By defaulty the Inter Character Timeout and the
     *          Inter Frame Delay are calculated automatically, based on the
     *          baud rate and the serial bit configuration.
     *          The Response Timeout and the Turnaround Delay have default
     *          values which can be found in SerialModbusConfig.h. */

    // uint32_t interCharacterTimeout = ModbusClient.getInterCharacterTimeout();
    // ModbusClient.setInterFrameDelay( interCharacterTimeout );

    // uint32_t interFrameDelay = ModbusClient.getInterFrameDelay();
    // ModbusClient.setInterCharacterTimeout( interFrameDelay );

    // uint32_t responseTimeout = ModbusClient.getResponseTimeout();
    // ModbusClient.setResponseTimeout( responseTimeout );

    // uint32_t turnaroundDelay = ModbusClient.getTurnaroundDelay();
    // ModbusClient.setTurnaroundDelay( turnaroundDelay );
}
/*----------------------------------------------------------------------------*/

void loop( void )
{
    MB_Status_t result = MB_OK;

    result = ModbusClient.setRequest( &requestMap[ 0 ] );
    if( result != MB_OK )
    {
        /* React to a possible request error. E.g. get an exception string based
         * on the result of the last operation on the Modbus object. */
        // Serial.println( ModbusClient.getExceptionString( result ) );
    }

    result = ModbusClient.process();
    if( result == MB_OK )
    {
        /* Instead of a callback function, we can also react here to the result
         * of a successful request. */
        if( data == 1 )
        {
            digitalWrite( LED_BUILTIN, HIGH );
        }
        else
        {
            digitalWrite( LED_BUILTIN, LOW );
        }
    }

    if( ModbusClient.setRequest( &requestMap[ 1 ] ) != MB_OK )
    {
        /* React to a possible request error. If you don't want to manage a
         * specific variable, you can get an exception string based on the
         * internally held status of the last operation. */
        // Serial.println( getLastExceptionString() );
    }

    if( ModbusClient.process() != MB_OK )
    {
        /* React to a possible request error. */
    }

    delay( 1000 );
}
/*----------------------------------------------------------------------------*/

void callback( void )
{
    if( data == 0 )
    {
        data = 1;
    }
    else
    {
        data = 0;
    }
}
