#include <SerialModbusSlave.h>


#define MAX485_CTRL_PIN 2

void max485Tx( void ){ digitalWrite( MAX485_CTRL_PIN, HIGH ); }
void max485Rx( void ){ digitalWrite( MAX485_CTRL_PIN, LOW ); }

#define ADDR_OBJ_01 0x0100
#define ADDR_OBJ_02 0x0200
#define ADDR_OBJ_03 0x0300
#define ADDR_OBJ_04 0x0400
#define ADDR_OBJ_05 0x0500

uint16_t obj_01[ 1 ] = { COIL_ON };
uint16_t obj_02[ 1 ] = { 20 };
uint16_t obj_03[ 3 ] = { 30, 31, 32 };
uint16_t obj_04[ 1 ] = { 40 };
uint16_t obj_05[ 5 ] = { 1, 2, 3, 4, 5 };
uint16_t obj_06[ 1 ] = { 60 };
uint16_t obj_07[ 1 ] = { 70 };

SerialModbusSlave slave;
    
MBData_t dataList[] = {
	/* Id   Function Code             Address      Data Object   Size */
    {  1,   WRITE_SINGLE_COIL,        ADDR_OBJ_01, obj_01,       1 },
    {  1,   READ_HOLDING_REGISTERS,   ADDR_OBJ_02, obj_02,       1 },
    {  1,   READ_INPUT_REGISTERS,     ADDR_OBJ_03, obj_03,       3 },
    {  1,   WRITE_SINGLE_REGISTER,    ADDR_OBJ_04, obj_04,       1 },
    {  1,   WRITE_MULTIPLE_REGISTERS, ADDR_OBJ_05, obj_05,       5 },
    
    /* You need to make a seperate entry if you want to make specific
     * addresses/registers readable and writeable.
     * With the following the objects 02 and 03 are writeable ... */
    {  1,   WRITE_SINGLE_REGISTER,    ADDR_OBJ_02, obj_02,       1 },
    {  1,   WRITE_SINGLE_REGISTER,    ADDR_OBJ_03, obj_03,       3 },
    /* ... and the objects 04 and 05 are readable. */
    {  1,   READ_INPUT_REGISTERS,     ADDR_OBJ_04, obj_04,       1 },
    {  1,   READ_INPUT_REGISTERS,     ADDR_OBJ_05, obj_05,       5 },
    
    /* The first element of an MBData_t entry holds the slave Id which
     * shoud be identical to all other entries.
     * But for special cases this stack is able to let a slave device
     * handle more than one slave Id.
     * The following two entries are for the vitual devices with the
     * Ids 2 and 3.*/
    {  2,   READ_HOLDING_REGISTERS,   ADDR_OBJ_02, obj_06,       1 },
    {  3,   READ_INPUT_REGISTERS,     ADDR_OBJ_03, obj_07,       1 },
    
	/* Data lists must be closed like this! */
    DATA_LIST_END
};

void setup()
{
	/* xxx */
    pinMode( MAX485_CTRL_PIN, OUTPUT );
    
    /* xxx */
    slave.setSerialCtrl( max485Tx, max485Rx );
    
    /* xxx */
    slave.setDataList( dataList );
    
    /* Initialize the Modbus Slave and the serial Baud Rate.
     * 
     * The boards standard serial Port is always set as default (e.g.
     * for the Arduino Uno: '&Serial'.)
     * If you use the Arduino Mega you can specifiy another port e.g.:
     * master.begin( 9600, &Serial2 );
     * 
     * You can also use an SoftwareSerial port. */
    slave.begin( 9600 );
    
	/******************************************************* OPTIONAL */
    
    /* xxx */
    slave.setProcessLoopHook( NULL );
    
    /******************************************************************/
    
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );
}

void loop()
{
	/* Now, with every loop the Stack will process one entry of the
	 * given data list ('requestList') successively and start over if
	 * the end of the list is reached. */
	MBStatus_t result = slave.processModbus();
	if( result != OK )
	{
		digitalWrite( LED_BUILTIN, HIGH );
	}
}
