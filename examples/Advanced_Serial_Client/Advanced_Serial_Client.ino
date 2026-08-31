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
    { 1, FC_WRITE_SINGLE_REGISTER,  1000, &data, 1, callback },
    { 1, FC_READ_HOLDING_REGISTERS, 2000, &data, 1, NULL },
    MB_REQUEST_MAP_END
};
/*----------------------------------------------------------------------------*/

void setup( void )
{
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );

    /* INFO:    The begin() method has two more parameters with default values.
     *          The following line shows the optional parameters to set the 
     *          serial interface we want to use and its configuration.
     *          e.g.: ModbusClient.begin( 9600, &Serial, SERIAL_8N1 ); */
    ModbusClient.begin( 9600 );

    /* The following functions can be used to get and set various timings.
     *
     * INFO:    By defaulty the Inter Character Timeout and the
     *          Inter Frame Delay are calculated automatically, based on the
     *          baud rate and the serial bit configuration.
     *          The Response Timeout and the Turnaround Delay have default
     *          values which can be found in SerialModbusConfig.h. */

    // uint32_t interCharacterTimeout = ModbusClient.getInterCharacterTimeout();
    // ModbusClient.setInterCharacterTimeout( interCharacterTimeout );

    // uint32_t interFrameDelay = ModbusClient.getInterFrameDelay();
    // ModbusClient.setInterFrameDelay( interFrameDelay );

    // uint32_t responseTimeout = ModbusClient.getResponseTimeout();
    // ModbusClient.setResponseTimeout( responseTimeout );

    // uint32_t turnaroundDelay = ModbusClient.getTurnaroundDelay();
    // ModbusClient.setTurnaroundDelay( turnaroundDelay );
}
/*----------------------------------------------------------------------------*/

void loop( void )
{
    MB_Status_t result = MB_OK;

    ModbusClient.setRequest( &requestMap[ 0 ] );
    result = ModbusClient.process();
    if( result != MB_OK )
    {
        /* React to a possible request error - e.g. in conjunction with one of
         * the following methods:
         *      - ModbusClient.getExceptionString( result );
         *      - ModbusClient.getLastExceptionString();
         *      - ModbusClient.getLastException(); */
    }

    ModbusClient.setRequest( &requestMap[ 1 ] );
    if( ModbusClient.process() == MB_OK )
    {
        /* Instead of a callback function, we can also react here to the result
         * of a successful request. */
        if( data == 0 )
        {
            data = 1;
        }
        else
        {
            data = 0;
        }
    }

    delay( 1000 );
}
/*----------------------------------------------------------------------------*/

void callback( void )
{
    if( data == 1 )
    {
        digitalWrite( LED_BUILTIN, HIGH );
    }
    else
    {
        digitalWrite( LED_BUILTIN, LOW );
    }
}
