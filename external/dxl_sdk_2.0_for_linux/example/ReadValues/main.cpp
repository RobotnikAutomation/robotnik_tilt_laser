/*
 * main.cpp
 *
 *  Created on: 2013. 12. 10.
 *      Author: chase
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <string.h>
#include <termios.h>
#include <math.h>
#include "dynamixel.h"

using namespace DXL_PRO;

Dynamixel DXL("/dev/ttyUSB0");

int _getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}


// Control table addresses
#define OPERATING_MODE     11
#define TORQUE_ENABLE      562
#define GOAL_POSITION      596
#define GOAL_TORQUE	   604
#define GOAL_VELOCITY	   600

#define PRESENT_CURRENT    621
#define PRESENT_VELOCITY   615
#define PRESENT_POSITION   611

// other settings
#define DEFAULT_BAUDNUM     1 // 57600 bps to match Dynamixel PRO's
#define CONTROL_PERIOD      (10) // arbitrary value


void PrintCommStatus(int CommStatus);
void PrintErrorCode(int ErrorCode);

int main()
{
    int GoalPos = 250000; // for 54-series
    //int GoalPos = 151875; // for 42-series
    int GoalVel = -3000;
    int GoalTorque = 120;
    int result = COMM_TXFAIL;
    int error = 0;
    int val1, val2, val3;
    // 1) declare USB2Dynamixel device
    Dynamixel DXL("/dev/ttyUSB0");

    // 2) Open USB2Dynamixel
    if( DXL.Connect() == 0 )
    {
        printf( "Failed to open USB2Dynamixel!\n" );
        printf( "Press any key to terminate...\n" );
        _getch();
        return 0;
    }
    else
        printf( "USB2Dynamixel initialized!\n" );

    // 3) set baud rate of USB2Dynamixel
    // In this case to match factory-default baud rate
    // of Dynamixel PRO
    if(DXL.SetBaudrate(DEFAULT_BAUDNUM) == true)
    {
    	printf( "Succeed to set USB2Dynamixel baud rate to 57600 bps!\n" );
    }
    else
    {
        printf( "Failed to change the baudrate!\n" );
        printf( "Press any key to terminate...\n" );
        _getch();
        return 0;
    }
    // 4) turn Torque Enable (562) off from ID 1 before changing Operating Mode
    
    // Torque Enable size is 1 byte 
    // WriteByte is the appropriate function
    DXL.WriteByte(1, TORQUE_ENABLE, 0, &error);
    usleep(CONTROL_PERIOD*100);    
   
    // 5) select Operating Mode and perform corresponding operation 
    printf( "Press\n" );
    printf( "a) 0 to select Torque Mode\n" );
    printf( "b) 1 to select Wheel Mode\n" );
    printf( "c) other press any other key to remain in Joint Mode\n\n" );
    printf( "Press ESC to stop operating\n" );
    printf( "Press CONTROL + C to exit\n" );
    while (1) {
    	switch(_getch()) {
	case 0x1b:
		DXL.WriteByte(1, TORQUE_ENABLE, 0, 0);
		usleep(CONTROL_PERIOD*100);
		DXL.WriteByte(1, OPERATING_MODE, 3, 0);
		break;
	case 0x30:
		DXL.WriteByte(1, OPERATING_MODE, 0, 0);
        	usleep(CONTROL_PERIOD*100);
		DXL.WriteByte(1, TORQUE_ENABLE, 1, 0);
        	usleep(CONTROL_PERIOD*100);
		// Goal Torque (604) size is 2 bytes (a word) 
		// WriteWord is the appropriate function
		DXL.WriteWord(1, GOAL_TORQUE, GoalTorque, &error);
		usleep(CONTROL_PERIOD*200);
		// after an arbitrary time (i.e. 2 secs in this case)
		// output onscreen
		// Present Current (621) size is 2 bytes (a word)
		// ReadWord is the appropriate function
		result = DXL.ReadWord(1, PRESENT_CURRENT, &val1, &error);
		if (result == COMM_RXSUCCESS)
			printf( "%d\n", val1);
		break;
	case 0x31:
		DXL.WriteByte(1, OPERATING_MODE, 1, 0);
        	usleep(CONTROL_PERIOD*100);
		DXL.WriteByte(1, TORQUE_ENABLE, 1, 0);
        	usleep(CONTROL_PERIOD*100);
		// Goal Velocity (600) size is 4 bytes (a double-word)
		// WriteDWrod is the appropriate function
		DXL.WriteDWord(1, GOAL_VELOCITY, GoalVel, &error);
		usleep(CONTROL_PERIOD*200);
		// after an arbitrary time (i.e. 2 secs in this case)
		// output onscreen
		// Present Velocity (615) size is 4 bytes (a double-word)
		// ReadDWord is the appropriate function
		result = DXL.ReadDWord(1, PRESENT_VELOCITY, (long*) &val2, &error);
		if (result == COMM_RXSUCCESS)
			printf( "%d\n", val2);
		break;
	default:		
		DXL.WriteByte(1, OPERATING_MODE, 3, 0);
		usleep(CONTROL_PERIOD*100);
        	DXL.WriteByte(1, TORQUE_ENABLE, 1, 0);
        	usleep(CONTROL_PERIOD*100);
		// Goal Position (596) size is 4 bytes (a double-word)
		// WriteDWrod is the appropriate function
		DXL.WriteDWord(1, GOAL_POSITION, GoalPos, &error);
		// after an arbitrary time (i.e. 2 secs in this case)
		// output onscreen
		// Present Position (611) size is 4 bytes (a double-word)
		// ReadDWord is the appropriate function
		usleep(CONTROL_PERIOD*200);
		result = DXL.ReadDWord(1, PRESENT_POSITION, (long*) &val3, &error);
		if (result == COMM_RXSUCCESS)
			printf( "%d\n", val3);
		break; }

    	}

    // 6) DIsconnect USB2Dynamixel before ending program
    DXL.Disconnect();

    return 0;
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
        printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
        break;

    case COMM_TXERROR:
        printf("COMM_TXERROR: Incorrect instruction packet!\n");
        break;

    case COMM_RXFAIL:
        printf("COMM_RXFAIL: Failed get status packet from device!\n");
        break;

    case COMM_RXWAITING:
        printf("COMM_RXWAITING: Now recieving status packet!\n");
        break;

    case COMM_RXTIMEOUT:
        printf("COMM_RXTIMEOUT: There is no status packet!\n");
        break;

    case COMM_RXCORRUPT:
        printf("COMM_RXCORRUPT: Incorrect status packet!\n");
        break;

    default:
        printf("This is unknown error code!\n");
        break;
    }
}

// Print error bit of status packet
void PrintErrorCode(int ErrorCode)
{
    if(ErrorCode & ERRBIT_VOLTAGE)
        printf("Input voltage error!\n");

    if(ErrorCode & ERRBIT_ANGLE)
        printf("Angle limit error!\n");

    if(ErrorCode & ERRBIT_OVERHEAT)
        printf("Overheat error!\n");

    if(ErrorCode & ERRBIT_RANGE)
        printf("Out of range error!\n");

    if(ErrorCode & ERRBIT_CHECKSUM)
        printf("Checksum error!\n");

    if(ErrorCode & ERRBIT_OVERLOAD)
        printf("Overload error!\n");

    if(ErrorCode & ERRBIT_INSTRUCTION)
        printf("Instruction code error!\n");
}


