#include <SerialModbusMaster.h>


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

SerialModbusMaster master;
    
MBData_t requestList[] = {
	/* Id   Function Code             Address      Data Object   Size */
    {  1,   WRITE_SINGLE_COIL,        ADDR_OBJ_01, obj_01,       1 },
    {  1,   READ_HOLDING_REGISTERS,   ADDR_OBJ_02, obj_02,       1 },
    {  1,   READ_INPUT_REGISTERS,     ADDR_OBJ_03, obj_03,       3 },
    {  1,   WRITE_SINGLE_REGISTER,    ADDR_OBJ_04, obj_04,       1 },
    {  1,   WRITE_MULTIPLE_REGISTERS, ADDR_OBJ_05, obj_05,       5 },
    
    /* It is also possible to read and write specific addresses inside
     * of the range of larger objects.
     * The following line will read only the 2nd element of 'obj_03'. */
	{ 1, READ_INPUT_REGISTERS, (ADDR_OBJ_03+1), obj_03, 1 },
	
	/* The first element of an MBData_t entry holds the slave Id of the
	 * desired slave device for the defined request. */
    { 2, READ_HOLDING_REGISTERS, ADDR_OBJ_02, obj_06, 1 },
    { 3, READ_INPUT_REGISTERS,   ADDR_OBJ_03, obj_07, 1 },
    
	/* Data lists must be closed like this! */
    DATA_LIST_END
};

void setup()
{
	/* xxx */
    pinMode( MAX485_CTRL_PIN, OUTPUT );
    
    /* xxx */
    master.setSerialCtrl( max485Tx, max485Rx );
    
    /* xxx */
    master.setDataList( requestList );
    
    /* Initialize the Modbus Master and the serial Baud Rate.
     * 
     * The boards standard serial Port is always set as default (e.g.
     * for the Arduino Uno: '&Serial'.)
     * If you use the Arduino Mega you can specifiy another port e.g.:
     * master.begin( 9600, &Serial2 );
     * 
     * You can also use an SoftwareSerial port. */
    master.begin( 9600 );
    
	/******************************************************* OPTIONAL */
    
    /* xxx */
    master.setResponseTimeout( 1000000 );
    
    /* xxx */
    master.setTurnaroundDelay( 200000 );
    
    /* xxx */
    master.setProcessLoopHook( NULL );
    
    /******************************************************************/
    
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, LOW );
}

void loop()
{
	/********************************************************** ( 1 ) */
	/* Now, with every loop the Stack will process one entry of the
	 * given data list ('requestList') successively and start over if
	 * the end of the list is reached. */
	MBStatus_t result = master.processModbus();
	if( result != OK )
	{
		digitalWrite( LED_BUILTIN, HIGH );
	}
	
	/*************************************** ONLY FOR THE EXAMPLE !!! */
	
	/* ATTENTION : This call is only placed here for this example!
	 *             In a normal situation it is not */
	master.setDataList( NULL );
	
	/********************************************************** ( 2 ) */
	
	/* It is also possible to use the 'setRequest()' method instead of
	 * 'setDataList()'. There for you have to use 'setRequest()' with
	 * a specific 'MBData_t' object address/pointer as parameter before
	 * you call 'processModbus()'. */
	MBData_t req = { 1, WRITE_SINGLE_COIL, ADDR_OBJ_01, obj_01, 1 };
	master.setRequest( &req );
	master.processModbus();
	
	/* Of course you can also use the address of a specific data list
	 * entry like the one defined here before ('requestList'). */
	master.setRequest( &requestList[ 1 ] );
	master.processModbus();
	
	/********************************************************** ( 3 ) */
	
	/* You can also set the data object 'NULL'. In this case you have
	 * to use 'getReplyDataSize()' and 'getReplyData()' right after
	 * 'processModbus()' because at the begining of  the next loop
	 * iteration the reply data will be deleted! */
	uint16_t buffer[ 20 ];
	req = { 1, READ_INPUT_REGISTERS, ADDR_OBJ_05, NULL, 1 };
	master.processModbus();
	int8_t replyDataSize = master.getReplyDataSize();
	if( replyDataSize > 0 )
	{
		master.getReplyData( buffer, 20 );
	}
	
	/******************************************************************/master.setDataList( requestList );
}
