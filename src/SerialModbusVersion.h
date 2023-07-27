////////////////////////////////////////////////////////////////////////////////
/**
 * @file        SerialModbusVersion.h
 * 
 * @author      Martin Legleiter
 * 
 * @brief       TODO
 * 
 * @copyright   (c) 2023 Martin Legleiter
 * 
 * @license     Use of this source code is governed by an MIT-style
 *              license that can be found in the LICENSE file or at
 *              @see https://opensource.org/licenses/MIT.
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_MODBUS_VERSION_H__
#define __SERIAL_MODBUS_VERSION_H__

/*-----------------------------------------------------------*/

#include <stdint.h>

/*-----------------------------------------------------------*/

#define SERIALMODBUS_VERSION_MAJOR      1
#define SERIALMODBUS_VERSION_MINOR      0
#define SERIALMODBUS_VERSION_PATCH      0

#define STR_HELPER( x )                 #x
#define STR( x )                        STR_HELPER( x )
#define SERIALMODBUS_VERSION_STR        STR( SERIALMODBUS_VERSION_MAJOR ) "."   \
                                        STR( SERIALMODBUS_VERSION_MINOR ) "."   \
                                        STR( SERIALMODBUS_VERSION_PATCH )

#define SERIALMODBUS_VERSION_HEX        ( ( ( uint32_t ) SERIALMODBUS_VERSION_MAJOR << 16 ) |   \
                                          ( ( uint32_t ) SERIALMODBUS_VERSION_MINOR <<  8 ) |   \
                                            ( uint32_t ) SERIALMODBUS_VERSION_PATCH           )

/*-----------------------------------------------------------*/

#define SERIALMODBUS_VERSION_CHK_EQ( major, minor, patch )  \
(                                                           \
    ( SERIALMODBUS_VERSION_MAJOR == ( major ) &&            \
      SERIALMODBUS_VERSION_MINOR == ( minor ) &&            \
      SERIALMODBUS_VERSION_PATCH == ( patch ) )             \
)
/*-----------------------------------------------------------*/

#define SERIALMODBUS_VERSION_CHK_NE( major, minor, patch )  \
(                                                           \
    !SERIALMODBUS_VERSION_CHK_EQ( major, minor, patch )     \
)
/*-----------------------------------------------------------*/

#define SERIALMODBUS_VERSION_CHK_LT( major, minor, patch )  \
(                                                           \
    ( SERIALMODBUS_VERSION_MAJOR <  ( major ) )             \
    ||                                                      \
    ( SERIALMODBUS_VERSION_MAJOR == ( major ) &&            \
      SERIALMODBUS_VERSION_MINOR <  ( minor ) )             \
    ||                                                      \
    ( SERIALMODBUS_VERSION_MAJOR == ( major ) &&            \
      SERIALMODBUS_VERSION_MINOR == ( minor ) &&            \
      SERIALMODBUS_VERSION_PATCH <  ( patch ) )             \
)
/*-----------------------------------------------------------*/

#define SERIALMODBUS_VERSION_CHK_GT( major, minor, patch )  \
(                                                           \
    ( SERIALMODBUS_VERSION_MAJOR >  ( major ) )             \
    ||                                                      \
    ( SERIALMODBUS_VERSION_MAJOR == ( major ) &&            \
      SERIALMODBUS_VERSION_MINOR >  ( minor ) )             \
    ||                                                      \
    ( SERIALMODBUS_VERSION_MAJOR == ( major ) &&            \
      SERIALMODBUS_VERSION_MINOR == ( minor ) &&            \
      SERIALMODBUS_VERSION_PATCH >  ( patch ) )             \
)
/*-----------------------------------------------------------*/

#define SERIALMODBUS_VERSION_CHK_LTOE( major, minor, patch )    \
(                                                               \
    SERIALMODBUS_VERSION_CHK_LT( major, minor, patch ) ||       \
    SERIALMODBUS_VERSION_CHK_EQ( major, minor, patch )          \
)
/*-----------------------------------------------------------*/

#define SERIALMODBUS_VERSION_CHK_GTOE( major, minor, patch )    \
(                                                               \
    SERIALMODBUS_VERSION_CHK_GT( major, minor, patch ) ||       \
    SERIALMODBUS_VERSION_CHK_EQ( major, minor, patch )          \
)
/*-----------------------------------------------------------*/

#endif /* __SERIAL_MODBUS_VERSION_H__ */
