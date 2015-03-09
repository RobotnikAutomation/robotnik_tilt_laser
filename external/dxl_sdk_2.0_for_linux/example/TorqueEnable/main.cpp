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


// Control table address
#define TORQUE_ENABLE      562

// Default settings
#define DEFAULT_BAUDNUM     1 // 57600 bps

void PrintCommStatus(int CommStatus);
void PrintErrorCode(int ErrorCode);

int main()
{
    int error = 0;
    int On = 1;
    int result;
    // 1) declare USB2Dynamixel
    Dynamixel DXL("/dev/ttyUSB0");

    // 2) Open USB2Dyanmixel
    if( DXL.Connect() == 0 )
    {
        printf( "Failed to open USB2Dynamixel!\n" );
        printf( "Press any key to end...\n" );
        _getch();
        return 0;
    }
    else
        printf( "Succeed to open USB2Dynamixel!\n" );


    if(DXL.SetBaudrate(DEFAULT_BAUDNUM) == true)
    {
    	printf( "Succeed to set USB2DXL baudrate to 57600 bps!\n" );
    }
    else
    {
        printf( "Failed to set USB2DXL baudrate to 57600 bps!\n" );
        printf( "Press any key to end ...\n" );
        _getch();
        return 0;
    }
    printf( "Turning Torque Enable on for ID 1\n" );
    // This step is essential change values on addresses located in the RAM area
    result = DXL.WriteByte(1, TORQUE_ENABLE, 1, &error); 
	
    // after enabling Torque Enable
    // add main routine involving RAM address(es) here (i.e. Goal Position)
    result =  DXL.ReadByte(1, TORQUE_ENABLE, &On, &error);
    if (result == COMM_RXSUCCESS)
    	printf( "%d\n", On);
    usleep(100);

    // Disconnect Dynamixel PRO
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


